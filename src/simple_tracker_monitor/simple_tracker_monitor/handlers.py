import tornado
import tornado.web
import tornado.websocket
from simple_tracker_interfaces.msg import TrackingState

class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):

  def set_extra_headers(self, path):
    # Disable cache
    self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class PrometheusMetricsHandler(tornado.web.RequestHandler):

  state:TrackingState = None

  def initialize(self):
    pass

  def get(self):
    self.write(f"<html><head></head><body><pre>{self.get_metrics()}<pre></body></html>")
  
  def get_metrics(self):
    metrics = f"Sky 360 metrics initialising"
    if PrometheusMetricsHandler.state != None:
      metrics = f"sky360_tracker_trackable_count {PrometheusMetricsHandler.state.trackable}\nsky360_tracker_alive_count {PrometheusMetricsHandler.state.alive}\nsky360_tracker_started_total {PrometheusMetricsHandler.state.started}\nsky360_tracker_ended_total {PrometheusMetricsHandler.state.ended}"
    return metrics
 
