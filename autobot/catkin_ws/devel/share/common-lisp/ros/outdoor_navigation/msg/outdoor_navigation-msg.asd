
(cl:in-package :asdf)

(defsystem "outdoor_navigation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "NavFlagMsg" :depends-on ("_package_NavFlagMsg"))
    (:file "_package_NavFlagMsg" :depends-on ("_package"))
    (:file "NavFlagMsg" :depends-on ("_package_NavFlagMsg"))
    (:file "_package_NavFlagMsg" :depends-on ("_package"))
  ))