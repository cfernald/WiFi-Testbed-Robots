
(cl:in-package :asdf)

(defsystem "traxxas_node-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AckermannMonitorMsg" :depends-on ("_package_AckermannMonitorMsg"))
    (:file "_package_AckermannMonitorMsg" :depends-on ("_package"))
    (:file "AckermannDriveMsg" :depends-on ("_package_AckermannDriveMsg"))
    (:file "_package_AckermannDriveMsg" :depends-on ("_package"))
    (:file "AckermannMonitorMsg" :depends-on ("_package_AckermannMonitorMsg"))
    (:file "_package_AckermannMonitorMsg" :depends-on ("_package"))
    (:file "AckermannDriveMsg" :depends-on ("_package_AckermannDriveMsg"))
    (:file "_package_AckermannDriveMsg" :depends-on ("_package"))
  ))