import sqlite3
from typing import List, Tuple, Union, Optional, Dict

class SQLiteManager:
    def __init__(self, db_path: str):
        """
        初始化数据库管理器
        :param db_path: SQLite 数据库文件路径
        """
        self.db_path = db_path

    def _connect(self):
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def create_table(self, table_name: str, columns: Dict[str, str]):
        with self._connect() as conn:
            cursor = conn.cursor()
            cols = ", ".join([f"{name} {dtype}" for name, dtype in columns.items()])
            sql = f"CREATE TABLE IF NOT EXISTS {table_name} ({cols})"
            cursor.execute(sql)
            conn.commit()

    def get_table_columns(self, table_name: str) -> List[str]:
        with self._connect() as conn:
            cursor = conn.cursor()
            cursor.execute(f"PRAGMA table_info({table_name})")
            return [row[1] for row in cursor.fetchall()]  # PRAGMA returns (cid, name, type...)

    def insert(self, table_name: str, data: Dict[str, Union[str, int, float]]) -> int:
        with self._connect() as conn:
            cursor = conn.cursor()
            keys = ", ".join(data.keys())
            placeholders = ", ".join(["?" for _ in data])
            values = list(data.values())
            sql = f"INSERT INTO {table_name} ({keys}) VALUES ({placeholders})"
            cursor.execute(sql, values)
            conn.commit()
            return cursor.rowcount

    def select(self, table_name: str, columns: List[str] = None, where: str = "", params: Tuple = ()) -> List[Dict]:
        with self._connect() as conn:
            cursor = conn.cursor()
            cols = ", ".join(columns) if columns else "*"
            sql = f"SELECT {cols} FROM {table_name}"
            if where:
                sql += f" WHERE {where}"
            cursor.execute(sql, params)
            rows = cursor.fetchall()
            return [dict(row) for row in rows]

    def update(self, table_name: str, data: Dict[str, Union[str, int, float]], where: str, params: Tuple):
        with self._connect() as conn:
            cursor = conn.cursor()
            set_clause = ", ".join([f"{k} = ?" for k in data.keys()])
            values = list(data.values()) + list(params)
            sql = f"UPDATE {table_name} SET {set_clause} WHERE {where}"
            cursor.execute(sql, values)
            conn.commit()

    def delete(self, table_name: str, where: str, params: Tuple):
        with self._connect() as conn:
            cursor = conn.cursor()
            sql = f"DELETE FROM {table_name} WHERE {where}"
            cursor.execute(sql, params)
            conn.commit()

    def get_tables(self) -> List[str]:
        with self._connect() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
            return [row[0] for row in cursor.fetchall()]

    def execute_raw(self, sql: str, params: Tuple = ()): 
        with self._connect() as conn:
            cursor = conn.cursor()
            cursor.execute(sql, params)
            conn.commit()

    def close(self):
        # 不再需要持久连接，因此留空
        pass