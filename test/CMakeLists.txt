add_executable(vtkBoxWidgetRestrictedTest vtkBoxWidgetRestrictedTest.cxx ${CMAKE_SOURCE_DIR}/vtkBoxWidgetRestricted.cpp)
target_link_libraries(vtkBoxWidgetRestrictedTest ${VTK_LIBRARIES})

add_executable(vtkAnnotationBoxSourceTest vtkAnnotationBoxSourceTest.cxx ${CMAKE_SOURCE_DIR}/vtkAnnotationBoxSource.cpp)
target_link_libraries(vtkAnnotationBoxSourceTest ${VTK_LIBRARIES})

QT5_WRAP_UI(ui_QVTKWidgetTest.h QVTKWidgetTest.ui)
add_executable(QVTKWidgetTest QVTKWidgetTest.cxx  ui_QVTKWidgetTest.h)
target_link_libraries(QVTKWidgetTest  ${PCL_LIBRARIES}  Qt5::Widgets)
