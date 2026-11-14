cmake_minimum_required(VERSION 3.8)
project(puma560_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf launch rviz
  DESTINATION share/${PROJECT_NAME})

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
