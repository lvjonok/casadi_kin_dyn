set(${PY_NAME}_SOURCES pyCasadiKinDyn.cpp)

# python3_add_library(${PY_NAME} MODULE WITH_SOABI ${${PY_NAME}_SOURCES})
message(STATUS ${PY_NAME})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
pybind11_add_module(${PY_NAME} pyCasadiKinDyn.cpp)
# target_link_libraries(${PY_NAME} PUBLIC cmeel-casadi_kin_dyn)
target_link_libraries(${PY_NAME} PUBLIC ${PROJECT_NAME})
target_link_libraries(
  ${PY_NAME}
  PUBLIC
  # cmeel-casadi-kin-dyn
  # PRIVATE
  pybind11::pybind11
  casadi
  ${PINOCCHIO_LIBRARIES}
  ${urdfdom_headers_LIBRARIES}
  ${urdfdom_LIBRARIES}
  ${Boost_LIBRARIES}
)
# target_link_libraries(
#   ${PROJECT_NAME} 
#   PUBLIC
#   # ${PINOCCHIO_LIBRARIES}
#   # casadi
#   # # PRIVATE
#   # ${urdfdom_headers_LIBRARIES}
#   # ${urdfdom_LIBRARIES}
#   # ${Boost_LIBRARIES}
# )
set_target_properties(${PY_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PY_NAME})
if(UNIX)
  get_relative_rpath(${${PY_NAME}_INSTALL_DIR} ${PY_NAME}_INSTALL_RPATH)
  set_target_properties(${PY_NAME} PROPERTIES INSTALL_RPATH
                                              "${${PY_NAME}_INSTALL_RPATH}")
endif()
install(TARGETS ${PY_NAME} DESTINATION ${${PY_NAME}_INSTALL_DIR})

set(${PY_NAME}_PYTHON __init__.py )

foreach(python ${${PY_NAME}_PYTHON})
  python_install_on_site(${PY_NAME} ${python})
endforeach()

# cmake_minimum_required(VERSION 3.5)

# find_package(pybind11 QUIET)

# add_compile_options(-std=c++17)

# # if(${pybind11_FOUND})
    
#     message(STATUS "Will compile python bindings")

#     include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#     set(MODULE_NAME pycasadi_kin_dyn)
#     if(${PYTHON_VERSION_MAJOR} EQUAL 3)
#         set(MODULE_NAME py3casadi_kin_dyn)
#     endif()

#     pybind11_add_module(${MODULE_NAME} pyCasadiKinDyn.cpp)
#     target_link_libraries(${MODULE_NAME} PUBLIC casadi_kin_dyn)
#     target_compile_definitions(${MODULE_NAME} PRIVATE "-DCASADI_KIN_DYN_MODULE=${MODULE_NAME}")

#     # get python install path 
#     execute_process(
#         COMMAND ${PYTHON_EXECUTABLE} -c "if True:
#             from distutils import sysconfig as sc
#             print(sc.get_python_lib(prefix='', plat_specific=True))"
#         OUTPUT_VARIABLE PYTHON_SITE
#         OUTPUT_STRIP_TRAILING_WHITESPACE)
#     message(STATUS "Python install dir: ${PYTHON_SITE}")
    
#     install(TARGETS ${MODULE_NAME}
#             DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})
            
#     install(FILES __init__.py DESTINATION ${PYTHON_SITE}/${PROJECT_NAME})

#     # Python package deployment
#     add_custom_target(generate_python_package
#         COMMAND mkdir -p ${PROJECT_NAME}
#         COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}
#         COMMAND ./copy_external_dependencies.py $<TARGET_FILE:${MODULE_NAME}> -d ${PROJECT_NAME} -c
#         COMMAND python3 -m build
#         DEPENDS ${MODULE_NAME}
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#         COMMENT "Generating package ${MODULE_NAME}"
#     )

#     add_custom_target(upload_python_package
#         COMMAND twine upload dist/*
#         DEPENDS generate_python_package
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#         COMMENT "Uploading package ${MODULE_NAME}"
#     )
      
# # else()
# #     message(STATUS "Pybind11 not found, bindings won't be available")
# # endif()
