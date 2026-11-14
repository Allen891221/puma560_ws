cmake_minimum_required(VERSION 3.8)
project(puma560_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf launch rviz
  DESTINATION share/${PROJECT_NAME})

# ---- 外觀 mesh：build 時依 dh_params.yaml 自動產生（上臂、前臂），確保尺寸永遠與 DH 一致 ----
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(MESH_DIR ${CMAKE_CURRENT_BINARY_DIR}/meshes)
set(MESHES ${MESH_DIR}/upper_arm.stl ${MESH_DIR}/forearm.stl)
add_custom_command(
  OUTPUT ${MESHES}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_meshes.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/dh_params.yaml ${MESH_DIR}
  DEPENDS scripts/generate_meshes.py config/dh_params.yaml
  COMMENT "產生 PUMA560 外觀 mesh")
add_custom_target(puma560_meshes ALL DEPENDS ${MESHES})
install(FILES ${MESHES} DESTINATION share/${PROJECT_NAME}/meshes)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # 驗證：展開後的 URDF joint 鏈乘積 = Craig DH 的 ⁰₆T
  # 不用 ament_add_pytest_test：它在設定階段執行 `pytest --version`，
  # 若 ~/.local 有不相容的 anyio pytest 外掛會失敗並「靜默跳過」測試，所以這裡直接呼叫 pytest。
  set(_result ${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}/test_urdf_matches_dh.xunit.xml)
  ament_add_test(test_urdf_matches_dh
    COMMAND python3 -m pytest -p no:anyio
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_urdf_matches_dh.py --junit-xml=${_result}
    RESULT_FILE ${_result})
endif()

ament_package()
