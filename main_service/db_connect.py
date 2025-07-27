import mysql.connector
from datetime import datetime

class ARCSDatabaseHandler:
    def __init__(self):
        self.conn = mysql.connector.connect(
            host="localhost",
            user="root",
            password="0303",
            database="ARCS_DB"
        )
        self.cursor = self.conn.cursor(dictionary=True)

    # ---------- USER ----------
    def save_user(self, data):
        sql = """
        INSERT INTO USER (name, boarding, departure, gate, sex, age, seat, from, to, ticket, robot_id, created)
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, NOW())
        """
        values = (
            data["name"], data["boarding"], data["departure"], data["gate"], data["sex"],
            data["age"], data["seat"], data["from"], data["to"], data["ticket"], data["robot_id"]
        )
        self.cursor.execute(sql, values)
        self.conn.commit()
        return "User saved."
    
    def get_user(self, name=None, ticket=None, robot_id=None):
        if name:
            self.cursor.execute("SELECT * FROM USER WHERE name = %s", (name,))
        elif ticket:
            self.cursor.execute("SELECT * FROM USER WHERE ticket = %s", (ticket,))
        elif robot_id:
            self.cursor.execute("SELECT * FROM USER WHERE robot_id = %s", (robot_id,))
        else:
            return "Name, Ticket or Robot ID must be provided."
        return self.cursor.fetchall()
    
    # ---------- ARCS ----------
    def save_arcs(self, data):
        sql = """
        REPLACE INTO ARCS (id, user, task, loading, using, battery, pos_x, pos_y, status_time)
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s, NOW())
        """
        values = (
            data["id"], data["user"], data["task"], data["loading"], data["using"],
            data["battery"], data["pos_x"], data["pos_y"]
        )
        self.cursor.execute(sql, values)
        self.conn.commit()
        return "ARCS info saved."
    
    def get_arcs(self, robot_id=None):
        if robot_id:
            self.cursor.execute("SELECT * FROM ARCS WHERE id = %s", (robot_id,))
        else:
            self.cursor.execute("SELECT * FROM ARCS")
        return self.cursor.fetchall()
    
    # ---------- route ----------
    def reset_route(self, robot_id, waypoints, destination):
        self.cursor.execute("DELETE FROM route WHERE robot_id = %s", (robot_id,))

        # 경유지 삽입
        for i, wp in enumerate(waypoints, start=1):
            self.cursor.execute("""
                INSERT INTO route (robot_id, id, type, pos_x, pos_y)
                VALUES (%s, %s, 'waypoint', %s, %s)
            """, (robot_id, i, wp["x"], wp["y"]))

        # 목적지 삽입
        self.cursor.execute("""
            INSERT INTO route (robot_id, id, type, pos_x, pos_y)
            VALUES (%s, %s, 'destination', %s, %s)
        """, (robot_id, len(waypoints) + 1, destination["x"], destination["y"]))
        self.conn.commit()
        return "Route reset."
    
    def delete_route_point(self, robot_id, point_type, x, y):
        self.cursor.execute("""
            DELETE FROM route
            WHERE robot_id = %s AND type = %s AND pos_x = %s AND pos_y = %s
        """, (robot_id, point_type, x, y))
        self.conn.commit()
        return "Route point deleted."
    
    def get_route(self, robot_id):
        self.cursor.execute("SELECT * FROM route WHERE robot_id = %s ORDER BY id ASC", (robot_id,))
        return self.cursor.fetchall()
    
    # ---------- admin ----------
    def verify_admin_login(self, admin_id, password):
        self.cursor.execute("SELECT * FROM admin WHERE id = %s AND password = %s", (admin_id, password))
        result = self.cursor.fetchone()
        return "Login success" if result else "Invalid credentials"
    
    def close(self):
        self.cursor.close()
        self.conn.close()
