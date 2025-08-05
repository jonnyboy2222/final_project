import pymysql
from datetime import datetime

class ARCSDatabaseHandler:
    def __init__(self):
        self.connection = pymysql.connect(
            host="localhost",
            user="root",
            password="password",
            db="ARCS_DB",
            port=3306,
            charset='utf8mb4',
            cursorclass=pymysql.cursors.DictCursor
        )

    # USER 저장
    def save_user(connection, user_data):
        query = """
            INSERT INTO USER (id, name, ticket, boarding, departure, gate, sex, age, seat, `from`, `to`, robot_id, created)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, NOW())
        """
        with connection.cursor() as cursor:
            cursor.execute(query, (
                user_data['id'], user_data['name'], user_data['ticket'], user_data['boarding'],
                user_data['departure'], user_data['gate'], user_data['sex'], user_data['age'],
                user_data['seat'], user_data['from'], user_data['to'], user_data['robot_id']
            ))
        connection.commit()

    # USER 조회
    def query_users(connection, robot_id=None, name=None, ticket=None, created=None):
        """
        입력된 조건(robot_id, name, ticket, created)에 따라 USER 테이블을 조회
        모두 None이면 전체 반환
        """
        base_query = "SELECT * FROM USER WHERE 1=1"
        conditions = []
        values = []
        if robot_id is not None:
            conditions.append("robot_id = %s")
            values.append(robot_id)
        if name is not None:
            conditions.append("name = %s")
            values.append(name)
        if ticket is not None:
            conditions.append("ticket = %s")
            values.append(ticket)
        if created is not None:
            conditions.append("DATE(created) = %s")
            values.append(created)
        query = base_query
        if conditions:
            query += " AND " + " AND ".join(conditions)
        with connection.cursor() as cursor:
            cursor.execute(query, values)
            results = cursor.fetchall()
        return results
    
    # USER ID 조회
    def get_latest_user_id_by_robot(self, robot_id):
        query = """
            SELECT id FROM USER
            WHERE robot_id = %s
            ORDER BY created DESC
            LIMIT 1
        """
        with self.connection.cursor() as cursor:
            cursor.execute(query, (robot_id,))
            result = cursor.fetchone()
            return result["id"] if result else None
        
    # ARCS 사용정보 업데이트 
    def update_arcs_status(self, robot_id, task_id, loading):
        user_id = self.get_latest_user_id_by_robot(robot_id)
        if not user_id:
            raise ValueError(f"No user found for robot_id = {robot_id}")
        table = f"ARCS_{robot_id}"
        query = f"""
            UPDATE {table}
            SET user = %s, task = %s, loading = %s, status_time = NOW()
            WHERE id = %s
        """
        with self.connection.cursor() as cursor:
            cursor.execute(query, (user_id, task_id, loading, robot_id))
        self.connection.commit()

    # ARCS 상태 업데이트 
    def update_arcs_position(connection, robot_id, pos_x, pos_y, battery):
        table = f"ARCS_{robot_id}"
        query = f"""
            UPDATE {table}
            SET pos_x = %s, pos_y = %s, battery = %s, status_time = NOW()
            WHERE id = %s
        """
        with connection.cursor() as cursor:
            cursor.execute(query, (pos_x, pos_y, battery, robot_id))
        connection.commit()

    # ROUTE 업데이트
    def replace_routes(connection, route_list):
        """
        route_list: list of dicts like:
        [{ "id": 1, "destination": "게이트 1번", "robot_id": 1 }, ...]
        """
        with connection.cursor() as cursor:
            cursor.execute("DELETE FROM route")
            query = "INSERT INTO route (id, destination, robot_id) VALUES (%s, %s, %s)"
            for route in route_list:
                cursor.execute(query, (route["id"], route["destination"], route["robot_id"]))
        connection.commit()

    
    def replace_routes(self, robot_id, route_points):
        """
        route_points: list of (x, y, theta)
        저장 순서대로 id = 1부터 자동 증가
        """
        with self.connection.cursor() as cursor:
            cursor.execute("DELETE FROM route WHERE robot_id = %s", (robot_id,))
            query = "INSERT INTO route (id, pos_x, pos_y, robot_id) VALUES (%s, %s, %s, %s)"
            for idx, (x, y, _) in enumerate(route_points, start=1):
                cursor.execute(query, (idx, x, y, robot_id))
        self.connection.commit()

    # 목적지 도착시 방문자 업데이트
    def increment_place_visitor(self, pos_x, pos_y):
        query = "UPDATE PLACE SET visitor = visitor + 1 WHERE pos_x = %s AND pos_y = %s"
        with self.connection.cursor() as cursor:
            cursor.execute(query, (pos_x, pos_y))
        self.connection.commit()

    # 자정 리셋 및 업데이트
    def reset_place_visitor_and_total(connection):
        with connection.cursor() as cursor:
            cursor.execute("UPDATE PLACE SET total = total + visitor")
            cursor.execute("UPDATE PLACE SET visitor = 0")
        connection.commit()

    # ADMIN 로그인 확인
    def verify_admin_login(self, username, password):
        """
        ADMIN 테이블에서 username과 password가 일치하는지 확인
        """
        query = "SELECT COUNT(*) AS count FROM admin WHERE username = %s AND password = %s"
        with self.connection.cursor() as cursor:
            cursor.execute(query, (username, password))
            result = cursor.fetchone()
            return result["count"] > 0
        
    # ARCS_# 조회
    def get_arcs(self, robot_id):
        """
        ARCS_#{robot_id} 테이블의 전체 상태를 조회
        """
        table_name = f"ARCS_{robot_id}"
        query = f"SELECT * FROM {table_name}"
        with self.connection.cursor() as cursor:
            cursor.execute(query)
            return cursor.fetchall()