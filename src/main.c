/*
 * Copyright (c) 2014-2017 Cesanta Software Limited
 * All rights reserved
 */

#include "mgos.h"
#include "mgos_rpc.h"

#define SAM3X_UART            1       // UART to use for communication with SAM3X
#define TIMEOUT_INITIAL       5000    // initial timeout waiting for first "OK", 100ms resolution
#define TIMEOUT_FOLLOWING     1000    // timeout waiting for "OK" between each line, 100ms resolution
#define MAX_GCODE_LINE_LENGTH 200     // max length of a GCODE line


struct state {
  struct mg_rpc_request_info *ri; /* RPC request info */
  int status;                     /* Request status */
  int64_t written;                /* Number of bytes written */
  FILE *fp;                       /* File to write to */
};

static struct state tstate;       // state variable for SAM3XDL request
static mgos_timer_id timer_id;    // timer id key


// HTTP callback

static void http_cb(struct mg_connection *c, int ev, void *ev_data, void *ud) {
  struct http_message *hm = (struct http_message *) ev_data;
  struct state *state = (struct state *) ud;

  switch (ev) {
    case MG_EV_CONNECT:
      state->status = *(int *) ev_data;
      break;
    case MG_EV_HTTP_CHUNK: {
      /*
       * Write data to file.
       */
      size_t n = fwrite(hm->body.p, 1, hm->body.len, state->fp);
      if (n != hm->body.len) {
        c->flags |= MG_F_CLOSE_IMMEDIATELY;
        state->status = 500;
      }
      state->written += n;
      c->flags |= MG_F_DELETE_CHUNK;
      break;
    }
    case MG_EV_HTTP_REPLY:
      /* Only when we successfully got full reply, set the status. */
      state->status = hm->resp_code;
      LOG(LL_INFO, ("Finished fetching"));
      c->flags |= MG_F_CLOSE_IMMEDIATELY;
      break;
    case MG_EV_CLOSE:
      LOG(LL_INFO, ("status %d bytes %llu", state->status, state->written));
      if (state->status == 200) {
        /* Report success only for HTTP 200 downloads */
        mg_rpc_send_responsef(state->ri, "{written: %llu}", state->written);
      } else {
        mg_rpc_send_errorf(state->ri, state->status, NULL);
      }
      if (state->fp != NULL) fclose(state->fp);
      free(state);
      break;
  }
}


// handler for Fetch RPC calls

static void fetch_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                          struct mg_rpc_frame_info *fi, struct mg_str args) {
  struct state *state;
  FILE *fp = NULL;
  char *url = NULL, *path = NULL;

  json_scanf(args.p, args.len, ri->args_fmt, &url, &path);

  if (url == NULL || (path == NULL)) {
    mg_rpc_send_errorf(ri, 500, "expecting url or file");
    goto done;
  }

  if (path != NULL && (fp = fopen(path, "w")) == NULL) {
    mg_rpc_send_errorf(ri, 500, "cannot open %s", path);
    goto done;
  }

  if ((state = calloc(1, sizeof(*state))) == NULL) {
    mg_rpc_send_errorf(ri, 500, "OOM");
    goto done;
  }

  state->fp = fp;
  state->ri = ri;

  LOG(LL_INFO, ("Fetching %s to %s", url, path));
  if (!mg_connect_http(mgos_get_mgr(), http_cb, state, url, NULL, NULL)) {
    free(state);
    mg_rpc_send_errorf(ri, 500, "malformed URL");
    goto done;
  }

  (void) cb_arg;
  (void) fi;

done:
  free(url);
  free(path);

}


/*****************************************************************************/

char gcode_buffer[MAX_GCODE_LINE_LENGTH + 5];
char response_buffer[20];
char *p = response_buffer;
static int timeout = TIMEOUT_INITIAL/10;


//
// timer callback, called every 100ms
//
static void timer_cb(void *arg)
{
  if (timeout > 0)        // if timeout is not expired
    timeout--;            // decrement count
  else
  {
    mgos_rpc_send_response(tstate.ri, "{\"Status\": \"TIMEOUT\"}"); // send timeout reply
    mgos_clear_timer(timer_id);                                     // stop timer
    mgos_uart_set_rx_enabled(SAM3X_UART, false);                    // disable RX callback
  }
  (void) arg;
}



//
// uart dispatcher, called when uart data is available
//
static void uart_dispatcher(int uart_no, void *arg)
{
  size_t rx_av = mgos_uart_read_avail(uart_no);     // get # of available characters
  if (rx_av > 0)                                    // if any available
  {
    struct mbuf rxb;
    mbuf_init(&rxb, 0);                             // init mbuf
    mgos_uart_read_mbuf(uart_no, &rxb, rx_av);      // read data from UART into mbuf
    if (rxb.len > 0)                                // if data was received
    {
      strncpy((char *) p, rxb.buf, rxb.len);        // copy data into receive line buffer
      p += rxb.len;                                 // increment buffer pointer

      if (strstr(response_buffer, "OK\r\n"))        // if we have a response match
      {
        // send next line
        if (fgets(gcode_buffer, MAX_GCODE_LINE_LENGTH, tstate.fp))  // try read a new line
        {
          strcat(gcode_buffer,"\r\n");                              // add CR/LF
          mgos_uart_write(SAM3X_UART, gcode_buffer, strlen(gcode_buffer));  // send line to UART
          mgos_uart_flush(SAM3X_UART);                              // flush UART to make sure data is sent immediately
          p = response_buffer;                                      // reset buffer pointer
          memset(response_buffer, 0, sizeof(response_buffer));      // clear buffer
        }
        else
        {
          // we're done
          fclose(tstate.fp);                        // close file
          mgos_rpc_send_response(tstate.ri, "{\"Status\": \"OK\"}");  // send reply
          mgos_clear_timer(timer_id);                                 // stop timer
          mgos_uart_set_rx_enabled(SAM3X_UART, false);                // disable RX callback
        }

        timeout = TIMEOUT_FOLLOWING/10;             // reset timeout
      }
    }
    mbuf_free(&rxb);                                // free mbuf
  }
  (void) arg;
}



//
// handler for SAM3XDL RPC calls
//
static void dl_handler(struct mg_rpc_request_info *ri, void *cb_arg,
                          struct mg_rpc_frame_info *fi, struct mg_str args) {
  FILE *fp = NULL;
  char *path = NULL;

  json_scanf(args.p, args.len, ri->args_fmt, &path);

  if (path == NULL) {
    mg_rpc_send_errorf(ri, 500, "expecting file");
    goto done;
  }

  if ((fp = fopen(path, "r")) == NULL) {
    mg_rpc_send_errorf(ri, 500, "cannot open %s", path);
    goto done;
  }

  p = response_buffer;                                  // reset buffer pointer
  memset(response_buffer, 0, sizeof(response_buffer));  // clear buffer

  tstate.fp = fp;
  tstate.ri = ri;

  if (fgets(gcode_buffer, MAX_GCODE_LINE_LENGTH, fp))
  {
    strcat(gcode_buffer,"\r\n");
    mgos_uart_write(SAM3X_UART, gcode_buffer, strlen(gcode_buffer));
    mgos_uart_flush(SAM3X_UART);
  }

  LOG(LL_INFO, ("Downloading %s to SAM3X", path));

  timer_id = mgos_set_timer(100 /* ms */, true /* repeat */, timer_cb, NULL /* arg */);
  timeout = TIMEOUT_INITIAL/10;

  mgos_uart_set_rx_enabled(SAM3X_UART, true);

  (void) cb_arg;
  (void) fi;

done:
  free(path);
}




enum mgos_app_init_result mgos_app_init(void) {

  mg_rpc_add_handler(mgos_rpc_get_global(), "Fetch",
                     "{url: %Q, file: %Q}", fetch_handler, NULL);

  mg_rpc_add_handler(mgos_rpc_get_global(), "SAM3XDL",
                     "{file: %Q}", dl_handler, NULL);


  // inialize UART

  struct mgos_uart_config ucfg;

  mgos_uart_config_set_defaults(SAM3X_UART, &ucfg);

  ucfg.baud_rate = 115200;
  ucfg.num_data_bits = 8;
  ucfg.parity = MGOS_UART_PARITY_NONE;
  ucfg.stop_bits = MGOS_UART_STOP_BITS_1;
  ucfg.rx_buf_size = 1500;
  ucfg.tx_buf_size = 1500;

  if (!mgos_uart_configure(SAM3X_UART, &ucfg)) {
    LOG(LL_ERROR, ("Failed to configure UART%d", SAM3X_UART));
  }

  mgos_uart_set_dispatcher(SAM3X_UART, uart_dispatcher, NULL);


  return MGOS_APP_INIT_SUCCESS;
}
