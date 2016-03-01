

(defsystem urdf-management
  :author "Jannik Buckelo <jannikbu@cs.uni-bremen.de>"
  :licence "BSD"

  :depends-on (:cl-urdf
               :roslisp
               :iai_urdf_msgs-srv
               :std_msgs-msg
               :s-xml)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "service" :depends-on ("package"))))))
