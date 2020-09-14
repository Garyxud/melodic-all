
qi_create_lib(alice_service SHARED
  SRC
    api.hpp
    imagestore.hpp
    imagestore.cpp
    imagestore_proxy.cpp
  DEPENDS QICORE
)
qi_stage_lib(alice_service)

if(QI_WITH_TESTS)
  qi_create_bin(file_example
    SRC
    file_example.cpp
    DEPENDS alice_service QICORE TESTSESSION
    )
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(TARGET alice_service PROPERTY FOLDER "file_example")
  set_property(TARGET file_example PROPERTY FOLDER "file_example")
endif()
