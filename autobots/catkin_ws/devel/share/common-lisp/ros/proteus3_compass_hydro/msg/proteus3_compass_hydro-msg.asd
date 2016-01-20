
(cl:in-package :asdf)

(defsystem "proteus3_compass_hydro-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CompassMsg" :depends-on ("_package_CompassMsg"))
    (:file "_package_CompassMsg" :depends-on ("_package"))
    (:file "CompassMsg" :depends-on ("_package_CompassMsg"))
    (:file "_package_CompassMsg" :depends-on ("_package"))
  ))