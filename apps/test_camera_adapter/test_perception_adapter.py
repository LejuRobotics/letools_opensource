#!/usr/bin/env python3
"""
感知适配器测试脚本

测试 PerceptionAdapter 的各项功能，包括：
- 适配器初始化
- 相机帧获取（RGB + 深度）
- 点云数据获取
- 相机状态查询
- AprilTag 检测
"""

import sys
import os
import time
import signal
from typing import Optional, Dict, Any

# 添加项目根目录到路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from adapters.hardware.leju_wheeled.perception_adapter import PerceptionAdapter
from adapters.hardware.leju_wheeled.camera_adapter import CameraAdapter

class PerceptionAdapterTest:
    """感知适配器测试类"""

    def __init__(self):
        self.adapter: Optional[PerceptionAdapter] = None
        self.camera_adapter: Optional[CameraAdapter] = None
        self.test_results: Dict[str, bool] = {}
        self.test_messages: Dict[str, str] = {}
        self._shutdown_flag = False
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """信号处理函数"""
        print("\n⚠️  收到终止信号，正在清理...")
        self._shutdown_flag = True
        if self.adapter:
            self.adapter.shutdown()
    
    def run_all_tests(self):
        """运行所有测试"""
        print("=" * 60)
        print("  PerceptionAdapter 功能测试")
        print("=" * 60)
        print()
        
        # 测试初始化
        self._test_initialization()
        
        # 测试相机帧获取
        self._test_camera_frame()
        
        # 测试深度数据获取
        self._test_depth_data()
        
        # 测试点云数据获取
        self._test_point_cloud()
        
        # 测试相机状态
        self._test_camera_status()
        
        # 测试 AprilTag 检测
        self._test_apriltag_detection()
        
        # 关闭适配器
        self._test_shutdown()
        
        # 输出测试报告
        self._print_report()
    
    def _test_initialization(self):
        """测试适配器初始化"""
        print("📦 测试1: 适配器初始化")
        try:
            # 先初始化 CameraAdapter（注入到 PerceptionAdapter）
            self.camera_adapter = CameraAdapter()
            camera_config = {
                'enable_head': True,
                'enable_wrist_camera': False,
            }
            camera_result = self.camera_adapter.initialize(camera_config)
            if not camera_result.success:
                print(f"   ❌ CameraAdapter 初始化失败: {camera_result.message}")
                self.test_results['initialization'] = False
                self.test_messages['initialization'] = camera_result.message
                return

            # 初始化 PerceptionAdapter（依赖注入）
            config = {
                'launch_apriltag': True
            }

            self.adapter = PerceptionAdapter()
            result = self.adapter.initialize(camera=self.camera_adapter, config=config)

            if result:
                print("   ✅ 初始化成功")
                self.test_results['initialization'] = True
                self.test_messages['initialization'] = "适配器初始化成功"
            else:
                print("   ❌ 初始化失败")
                self.test_results['initialization'] = False
                self.test_messages['initialization'] = "适配器初始化失败"

        except Exception as e:
            print(f"   ❌ 初始化异常: {str(e)}")
            self.test_results['initialization'] = False
            self.test_messages['initialization'] = f"初始化异常: {str(e)}"

        print()
    
    def _test_camera_frame(self):
        """测试相机帧获取"""
        print("📷 测试2: 相机帧获取")
        if not self.adapter:
            print("   ⚠️  跳过（适配器未初始化）")
            self.test_results['camera_frame'] = None
            self.test_messages['camera_frame'] = "跳过（适配器未初始化）"
            print()
            return
        
        try:
            print("   等待相机数据...")
            frame = None
            timeout = 10
            start_time = time.time()
            
            while frame is None and (time.time() - start_time) < timeout and not self._shutdown_flag:
                frame = self.adapter.get_camera_frame("camera")
                time.sleep(0.5)
            
            if frame is not None:
                color_shape = frame.color_image.shape if frame.color_image is not None else "None"
                depth_shape = frame.depth_image.shape if frame.depth_image is not None else "None"
                print(f"   ✅ 获取成功")
                print(f"      RGB图像: {color_shape}")
                print(f"      深度图像: {depth_shape}")
                print(f"      时间戳: {frame.timestamp}")
                self.test_results['camera_frame'] = True
                self.test_messages['camera_frame'] = f"RGB: {color_shape}, Depth: {depth_shape}"
            else:
                print("   ❌ 获取失败（超时）")
                self.test_results['camera_frame'] = False
                self.test_messages['camera_frame'] = "获取失败（超时）"
                
        except Exception as e:
            print(f"   ❌ 获取异常: {str(e)}")
            self.test_results['camera_frame'] = False
            self.test_messages['camera_frame'] = f"获取异常: {str(e)}"
        
        print()
    
    def _test_depth_data(self):
        """测试深度数据获取"""
        print("📊 测试3: 深度数据获取")
        if not self.adapter:
            print("   ⚠️  跳过（适配器未初始化）")
            self.test_results['depth_data'] = None
            self.test_messages['depth_data'] = "跳过（适配器未初始化）"
            print()
            return
        
        try:
            depth_data = None
            timeout = 10
            start_time = time.time()
            
            while depth_data is None and (time.time() - start_time) < timeout and not self._shutdown_flag:
                depth_data = self.adapter.get_depth_data("camera")
                time.sleep(0.5)
            
            if depth_data is not None:
                depth_shape = depth_data.depth_image.shape
                print(f"   ✅ 获取成功")
                print(f"      深度图像: {depth_shape}")
                print(f"      缩放因子: {depth_data.scale}")
                print(f"      时间戳: {depth_data.timestamp}")
                self.test_results['depth_data'] = True
                self.test_messages['depth_data'] = f"深度图像: {depth_shape}"
            else:
                print("   ⚠️  未获取到深度数据（可能需要硬件）")
                self.test_results['depth_data'] = None
                self.test_messages['depth_data'] = "未获取到深度数据"
                
        except Exception as e:
            print(f"   ❌ 获取异常: {str(e)}")
            self.test_results['depth_data'] = False
            self.test_messages['depth_data'] = f"获取异常: {str(e)}"
        
        print()
    
    def _test_point_cloud(self):
        """测试点云数据获取"""
        print("☁️  测试4: 点云数据获取")
        if not self.adapter:
            print("   ⚠️  跳过（适配器未初始化）")
            self.test_results['point_cloud'] = None
            self.test_messages['point_cloud'] = "跳过（适配器未初始化）"
            print()
            return
        
        try:
            pc_data = None
            timeout = 15
            start_time = time.time()
            
            while pc_data is None and (time.time() - start_time) < timeout and not self._shutdown_flag:
                pc_data = self.adapter.get_point_cloud("camera")
                time.sleep(0.5)
            
            if pc_data is not None:
                num_points = pc_data.points.shape[0] if pc_data.points.size > 0 else 0
                has_colors = pc_data.colors is not None
                print(f"   ✅ 获取成功")
                print(f"      点云数量: {num_points}")
                print(f"      包含颜色信息: {has_colors}")
                print(f"      坐标系: {pc_data.frame_id}")
                self.test_results['point_cloud'] = True
                self.test_messages['point_cloud'] = f"{num_points} 个点"
            else:
                print("   ⚠️  未获取到点云数据（可能需要硬件）")
                self.test_results['point_cloud'] = None
                self.test_messages['point_cloud'] = "未获取到点云数据"
                
        except Exception as e:
            print(f"   ❌ 获取异常: {str(e)}")
            self.test_results['point_cloud'] = False
            self.test_messages['point_cloud'] = f"获取异常: {str(e)}"
        
        print()
    
    def _test_camera_status(self):
        """测试相机状态查询"""
        print("📈 测试5: 相机状态查询")
        if not self.adapter:
            print("   ⚠️  跳过（适配器未初始化）")
            self.test_results['camera_status'] = None
            self.test_messages['camera_status'] = "跳过（适配器未初始化）"
            print()
            return
        
        try:
            status = self.adapter.get_camera_status("camera")
            
            if status is not None:
                print(f"   ✅ 查询成功")
                print(f"      运行状态: {'运行中' if status.is_running else '已停止'}")
                print(f"      帧计数: {status.frame_count}")
                print(f"      帧率: {status.fps:.1f} FPS")
                self.test_results['camera_status'] = True
                self.test_messages['camera_status'] = f"运行中, {status.frame_count} 帧, {status.fps:.1f} FPS"
            else:
                print("   ❌ 查询失败")
                self.test_results['camera_status'] = False
                self.test_messages['camera_status'] = "查询失败"
                
        except Exception as e:
            print(f"   ❌ 查询异常: {str(e)}")
            self.test_results['camera_status'] = False
            self.test_messages['camera_status'] = f"查询异常: {str(e)}"
        
        print()
    
    def _test_apriltag_detection(self):
        """测试 AprilTag 检测"""
        print("🔍 测试6: AprilTag 检测")
        if not self.adapter:
            print("   ⚠️  跳过（适配器未初始化）")
            self.test_results['apriltag'] = None
            self.test_messages['apriltag'] = "跳过（适配器未初始化）"
            print()
            return
        
        try:
            print("   等待检测结果（请将二维码对准相机）")
            tags = None
            timeout = 15
            start_time = time.time()
            
            while not tags and (time.time() - start_time) < timeout and not self._shutdown_flag:
                tags = self.adapter.get_tag_detections()
                time.sleep(0.5)
            
            if tags and len(tags) > 0:
                print(f"   ✅ 检测到 {len(tags)} 个二维码")
                for tag in tags:
                    print(f"      ID: {tag.tag_id}, 位置: ({tag.pose_in_world.x:.2f}, {tag.pose_in_world.y:.2f}, {tag.pose_in_world.z:.2f})")
                self.test_results['apriltag'] = True
                self.test_messages['apriltag'] = f"检测到 {len(tags)} 个二维码"
            else:
                print("   ⚠️  未检测到二维码（请确保二维码在视野内）")
                self.test_results['apriltag'] = None
                self.test_messages['apriltag'] = "未检测到二维码"
                
        except Exception as e:
            print(f"   ❌ 检测异常: {str(e)}")
            self.test_results['apriltag'] = False
            self.test_messages['apriltag'] = f"检测异常: {str(e)}"
        
        print()
    
    def _test_shutdown(self):
        """测试适配器关闭"""
        print("🔌 测试7: 适配器关闭")
        if not self.adapter:
            print("   ⚠️  跳过（适配器未初始化）")
            self.test_results['shutdown'] = None
            self.test_messages['shutdown'] = "跳过（适配器未初始化）"
            print()
            return

        try:
            self.adapter.shutdown()
            if self.camera_adapter:
                self.camera_adapter.shutdown()
            print("   ✅ 关闭成功")
            self.test_results['shutdown'] = True
            self.test_messages['shutdown'] = "关闭成功"

        except Exception as e:
            print(f"   ❌ 关闭异常: {str(e)}")
            self.test_results['shutdown'] = False
            self.test_messages['shutdown'] = f"关闭异常: {str(e)}"

        print()
    
    def _print_report(self):
        """输出测试报告"""
        print("=" * 60)
        print("  测试报告")
        print("=" * 60)
        
        passed = sum(1 for v in self.test_results.values() if v is True)
        failed = sum(1 for v in self.test_results.values() if v is False)
        skipped = sum(1 for v in self.test_results.values() if v is None)
        total = len(self.test_results)
        
        print(f"\n测试统计: {passed} 通过, {failed} 失败, {skipped} 跳过")
        print(f"通过率: {passed}/{total} ({passed/total*100:.1f}%)")
        
        print("\n详细结果:")
        for test_name, result in self.test_results.items():
            if result is True:
                status = "✅ 通过"
            elif result is False:
                status = "❌ 失败"
            else:
                status = "⚠️  跳过"
            message = self.test_messages.get(test_name, "")
            print(f"  {test_name}: {status} - {message}")
        
        print("\n" + "=" * 60)
        
        # 返回退出码
        if failed > 0:
            sys.exit(1)
        else:
            sys.exit(0)

if __name__ == '__main__':
    test = PerceptionAdapterTest()
    test.run_all_tests()
