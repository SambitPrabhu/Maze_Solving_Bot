;; Auto-generated. Do not edit!


(when (boundp 'package1234::dest)
  (if (not (find-package "PACKAGE1234"))
    (make-package "PACKAGE1234"))
  (shadow 'dest (find-package "PACKAGE1234")))
(unless (find-package "PACKAGE1234::DEST")
  (make-package "PACKAGE1234::DEST"))

(in-package "ROS")
;;//! \htmlinclude dest.msg.html


(defclass package1234::dest
  :super ros::object
  :slots (_dest_x_coordinate _dest_y_coordinate ))

(defmethod package1234::dest
  (:init
   (&key
    ((:dest_x_coordinate __dest_x_coordinate) 0.0)
    ((:dest_y_coordinate __dest_y_coordinate) 0.0)
    )
   (send-super :init)
   (setq _dest_x_coordinate (float __dest_x_coordinate))
   (setq _dest_y_coordinate (float __dest_y_coordinate))
   self)
  (:dest_x_coordinate
   (&optional __dest_x_coordinate)
   (if __dest_x_coordinate (setq _dest_x_coordinate __dest_x_coordinate)) _dest_x_coordinate)
  (:dest_y_coordinate
   (&optional __dest_y_coordinate)
   (if __dest_y_coordinate (setq _dest_y_coordinate __dest_y_coordinate)) _dest_y_coordinate)
  (:serialization-length
   ()
   (+
    ;; float64 _dest_x_coordinate
    8
    ;; float64 _dest_y_coordinate
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _dest_x_coordinate
       (sys::poke _dest_x_coordinate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dest_y_coordinate
       (sys::poke _dest_y_coordinate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _dest_x_coordinate
     (setq _dest_x_coordinate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dest_y_coordinate
     (setq _dest_y_coordinate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get package1234::dest :md5sum-) "cf363d99a20f66b06e0d4259cb1930ec")
(setf (get package1234::dest :datatype-) "package1234/dest")
(setf (get package1234::dest :definition-)
      "float64 dest_x_coordinate
float64 dest_y_coordinate

")



(provide :package1234/dest "cf363d99a20f66b06e0d4259cb1930ec")


