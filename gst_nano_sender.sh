
# Preview depth video on screen
# Send color video as RTSP stream over UDP


# IP Address of the this machine hosting the UDP stream
IP_ADDRESS=192.168.100.33

# Address and port to serve the video stream; check to make sure ports are available and firewalls don't block it!
TCP_SINK="tcpserversink host=$IP_ADDRESS port=8554"
UDP_SINK="udpsink host=$IP_ADDRESS port=8554"

#show gst-launch on the command line; can be useful for debugging
echo gst-launch-1.0 -vvv -e \
   realsensesrc serial=$SERIAL timestamp-mode=clock_all enable-color=true color-height=240 color-width=424 depth-height=240 depth-width=424 ! rgbddemux name=demux \
demux.src_color ! queue ! videoconvert ! omxh264enc ! video/x-h264, stream-format=byte-stream ! rtph264pay pt=96 ! $UDP_SINK \
demux.src_depth ! queue ! vv ! videoconvert ! omxh264enc ! video/x-h264, stream-format=byte-stream ! rtph264pay pt=96 ! udpsink host=$IP_ADDRESS port=8564					\
 
# first queue is for the preview
# third queue sends H.264 in MPEG container over UDP

source /opt/aivero/rgbd_toolkit/aivero_environment.sh

export SERIAL=936322070874
gst-launch-1.0 -vvv -e \
   realsensesrc serial=$SERIAL timestamp-mode=clock_all enable-color=true color-height=240 color-width=424 depth-height=240 depth-width=424 ! rgbddemux name=demux \
demux.src_color ! queue ! videoconvert ! omxh264enc ! video/x-h264, stream-format=byte-stream ! rtph264pay pt=96 ! $UDP_SINK \
demux.src_depth ! queue ! vv ! videoconvert ! omxh264enc ! video/x-h264, stream-format=byte-stream ! rtph264pay pt=96 ! udpsink host=$IP_ADDRESS port=8564\

  # rgbddemux name=demux \
 #  demux.src_color ! queue ! videoconvert ! glimagesink \
   #demux.src_depth ! queue ! colorizer near-cut=100 far-cut=2000 ! videoconvert ! glimagesink \
