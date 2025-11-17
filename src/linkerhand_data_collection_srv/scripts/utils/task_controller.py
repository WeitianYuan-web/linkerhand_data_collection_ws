#!/usr/bin/env python3
# -*-coding:utf8-*-
import os,sys,time,json
import numpy as np
from utils.db.sqlite_manager import SQLiteManager
'''
table: task
columns: ['id', 'task_id', 'task_name', 'task_path', 'task_configs_name', 'create_time', 'state']
'''
class TaskController:
    def __init__(self):
        self.sql = SQLiteManager(os.path.dirname(os.path.abspath(__file__))+"/db/linkerhand_aloha_database.db")
        #self.tables = self.db.get_tables()
        # Ensure required tables exist for fresh clones
        try:
            self.sql.create_table("task", {
                "id": "INTEGER PRIMARY KEY AUTOINCREMENT",
                "task_id": "INTEGER DEFAULT 0",
                "task_name": "TEXT NOT NULL",
                "task_path": "TEXT NOT NULL",
                "task_configs_name": "TEXT NOT NULL",
                "create_time": "TEXT NOT NULL",
                "state": "INTEGER NOT NULL",
                "task_id": "INTEGER"
            })
        except Exception as e:
            print(f"初始化数据库表失败: {e}")
        
        
    def _is_empty_value(self,value) -> bool:
        """判断一个值是否为空（自定义标准）"""
        if value is None:
            return True
        if isinstance(value, (str, list, dict, tuple)) and len(value) == 0:
            return True
        return False

    def _has_empty_value(self,d: dict) -> bool:
        """判断字典中是否有任何空值"""
        return any(self._is_empty_value(v) for v in d.values())

    def _get_empty_keys(self,d: dict) -> list:
        """获取字典中值为空的 key 列表"""
        return [k for k, v in d.items() if self._is_empty_value(v)]
    
    def write_data(self, table_name: str, data: dict) -> int:
        """
        插入记录
        :param table_name: 表名
        :param data: 要插入的数据字典
        :return: 插入的行数
        """
        if self._has_empty_value(data):
            empty_keys = self._get_empty_keys(data)
            print(f"插入失败，以下键的值为空：{empty_keys}")
            return 0
        rowcount = self.sql.insert(table_name, data)
        return rowcount
    
    def get_data(self, table_name: str, columns: list = None, where: str = "", params: tuple = ()) -> list:
        """
        查询记录
        :param table_name: 表名
        :param columns: 要查询的列名列表
        :param where: WHERE 子句
        :param params: WHERE 子句的参数
        :return: 查询结果列表
        self.task.get_data("task",["task_path"],f"task_name='{task_name}' and task_id={task_id}")
        """
        data = self.sql.select(table_name, columns, where, params)
        return data