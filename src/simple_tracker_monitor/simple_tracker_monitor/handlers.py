import tornado
import tornado.web
import tornado.websocket

class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):

  def set_extra_headers(self, path):
    # Disable cache
    self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class PrometheusMetricsHandler(tornado.web.RequestHandler):

  def initialize(self, node):
    self.node = node
    pass

  def get(self):
    #self.write(f"<html><head></head><body><pre>Sky360 Initialising<pre></body></html>")
    self.write(f"<html><head></head><body><pre>{self.node.get_metrics()}<pre></body></html>")
    
