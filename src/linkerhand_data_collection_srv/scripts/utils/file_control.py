#!/usr/bin/env python3
# -*-coding:utf8-*-
import os,sys,time,json
from obs import ObsClient, GetObjectHeader
from obs import PutObjectHeader,HeadPermission
from typing import Optional
import traceback

from .config_loader import DATA_DIR
from utils.color_msg import ColorMsg

class FileControl:
    def __init__(self):
        self.obsClient = None
        
    def _get_obs_client(self):
        """Lazy initialization of OBS client"""
        if self.obsClient is None:
            self.obsClient = ObsClient(access_key_id="FWWWLSFYV8VKJXNBPLU5", secret_access_key="0LLJ6IdkV4hnJEUKXa1LBZi4F18Ir5rZdCZS3KOI", server="https://obs.cn-north-4.myhuaweicloud.com")
        return self.obsClient

    def progress_callback(self, consumed_bytes, total_bytes):
        """Progress callback for OBS upload"""
        if total_bytes:
            rate = int(100 * consumed_bytes / total_bytes)
            print(f"\rUpload progress: {rate}%", end='', flush=True)
            if rate == 100:
                print()  # New line when complete

    # 创建文件和文件夹mqtt用
    def create_directory_tree(self, base_path=DATA_DIR, sub_dir_path='', file: Optional[str] = None):
        """
        动态递归创建目录，并在最后一级目录内创建一个文件（如果文件不存在）。
        
        :param base_path: 根目录路径（字符串）。
        :param sub_dir_path: 子目录路径（字符串）。
        :param file: 要创建的文件名（字符串）。
        """
        # 拼接完整路径
        #full_path = os.path.join(base_path, sub_dir_path)
        full_path = base_path + "/" +sub_dir_path
        print(base_path)
        # 分解路径为逐级目录
        parts = full_path.split(os.sep)

        current_path = parts[0] if parts[0] else os.sep  # 处理绝对路径的根目录
        tar = 0
        for part in parts[1:]:
            current_path = os.path.join(current_path, part)
            if not os.path.exists(current_path):
                try:
                    os.makedirs(current_path)
                    print(f"目录已创建: {current_path}")
                except PermissionError:
                    print(f"权限不足，无法创建目录: {current_path}")
                    return 99
                except Exception as e:
                    print(f"创建目录失败: {e}")
                    return 99
            else:
                print(f"目录已存在: {current_path}")
        if file != None:
            # 在最后一级目录中创建文件
            task_file_path = os.path.join(full_path, file)
            if os.path.exists(task_file_path):
                print(f"文件已存在: {task_file_path}，不创建新文件")
                return 0
            else:
                try:
                    with open(task_file_path, "w") as file:
                        file.write("")  # 创建空文件
                    print(f"文件已创建: {task_file_path}")
                    return 1
                except PermissionError:
                    print(f"权限不足，无法创建文件: {task_file_path}")
                    return 99
                except Exception as e:
                    print(f"创建文件失败: {e}")
                    return 99
        else:
            return 1
        

    # 遍历指定目录下所有文件和文件夹
    def list_directory_to_json(self,directory_path=DATA_DIR):
        """
        遍历指定目录下的所有文件和文件夹，并返回 JSON 格式的数据。
        :param directory_path: 需要遍历的目录路径。
        :return: JSON 格式的字符串，包含文件和文件夹的层级结构。
        """
        def traverse_directory(path):
            """
            递归遍历目录，构建文件夹和文件的层级结构。
            :param path: 当前目录路径。
            :return: 字典，表示当前目录的层级结构。
            """
            structure = {"name": os.path.basename(path), "type": "directory", "children": []}
            try:
                for entry in os.listdir(path):
                    if entry != "":
                        entry_path = os.path.join(path, entry)
                        if os.path.isdir(entry_path):
                            # 如果是文件夹，递归调用
                            structure["children"].append(traverse_directory(entry_path))
                        elif os.path.isfile(entry_path):
                            # 如果是文件，直接添加
                            structure["children"].append({"name": entry, "type": "file"})
            except PermissionError:
                # 处理权限不足的问题
                structure["children"].append({"name": "Permission Denied", "type": "error"})
            return structure

        if not os.path.exists(directory_path):
            return json.dumps({"error": "Directory does not exist"})

        # 构建层级结构
        result = traverse_directory(directory_path)
        return result

    
    def up_load(self, path="",datasetPath="",file_name="",folder_name="",sn=""):
        try:
            # 上传对象的附加头域
            headers = PutObjectHeader()
            # 【可选】待上传对象的MIME类型
            headers.contentType = 'text/plain'
            # bucketName = "columbus-robot"
            bucketName = "columbus-robot-dev"
            objectKey = datasetPath +"/"+path
            path = DATA_DIR + path
            ColorMsg(msg=f"上传的文件夹：{path}", color="blue")
            # 上传文件的自定义元数据
            #metadata = {'meta1': 'value1', 'meta2': 'value2'}
            metadata = {}
            # 文件上传
            #resp = self._get_obs_client().putFile(bucketName, objectKey, path, metadata, headers)
            print("path:",path)
            print("objectKey:",objectKey)
            print("datasetPath:",datasetPath)
            print("bucketName:",bucketName)
            resp = self._get_obs_client().putFile(bucketName, objectKey, path, progressCallback=self.progress_callback)
            
            # 返回码为2xx时，接口调用成功，否则接口调用失败
            if isinstance(resp, list):
                self.out_put_res(resp=resp)
            else:
                if resp.status < 300:
                    print('Put File Succeeded')
                    print('requestId:', resp.requestId)
                    print('etag:', resp.body.etag)
                    etag = resp.body.etag
                    print('objectUrl:', resp.body.objectUrl)
                    objectUrl = resp.body.objectUrl
                    print('versionId:', resp.body.versionId)
                    print('storageClass:', resp.body.storageClass)
                else:
                    print('Put File Failed')
                    print('requestId:', resp.requestId)
                    print('errorCode:', resp.errorCode)
                    print('errorMessage:', resp.errorMessage)
                    objectUrl = "Error"
        except:
            print('Put File Failed')
            print(traceback.format_exc())
            return 9, "Error"

        dic = {
            "method":"upload_result",
            "id":121212,
            "params":{
                "upload_status_code":1,
                "msg":"SUCCESS",
                "folder_name":folder_name,
                "SN":sn,
                "file_url":objectUrl,
                "file_name":file_name
            }
        }

        return dic, objectUrl


