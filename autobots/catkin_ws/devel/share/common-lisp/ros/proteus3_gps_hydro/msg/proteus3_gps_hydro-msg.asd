
(cl:in-package :asdf)

(defsystem "proteus3_gps_hydro-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GPSMsg" :depends-on ("_package_GPSMsg"))
    (:file "_package_GPSMsg" :depends-on ("_package"))
    (:file "GPSMsg" :depends-on ("_package_GPSMsg"))
    (:file "_package_GPSMsg" :depends-on ("_package"))
  ))