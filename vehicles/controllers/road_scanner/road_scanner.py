from controller import Supervisor

robot = Supervisor()

def lerp(p1, p2, t):
    return [p1[i] + (p2[i] - p1[i]) * t for i in range(len(p1))]

print("--- [Road_Scanner] 전 구간 XZ 정밀 스캔 시작 ---")
save_path = "/Users/baggyeongsu/Documents/vehicles/road_map.txt"

try:
    # [필독] Scene Tree에서 확인한 도로 DEF 이름을 연결 순서대로 모두 넣으세요!
    road_sequence = ["MIDDLE_SOUTH", "SOUTH_UPPER_INPUT", "SOUTH_UPPER_LOOP", "CURVE", "SLOPE"] 
    
    points = []
    for name in road_sequence:
        node = robot.getFromDef(name)
        if node:
            pos = node.getPosition() # 절대 좌표 [x, y, z]
            points.append([pos[0], pos[2]]) # X, Z만 저장
            print(f"✅ {name} 좌표 확보")

    if len(points) > 1:
        points.append(points[0]) # 무한 루프 주행을 위해 시작점 연결
        with open(save_path, "w") as f:
            total = 0
            for i in range(len(points) - 1):
                for j in range(2000): # 조각당 2000개로 쪼갬
                    px = points[i][0] + (points[i+1][0] - points[i][0]) * (j/2000.0)
                    pz = points[i][1] + (points[i+1][1] - points[i][1]) * (j/2000.0)
                    f.write(f"{px:.6f},{pz:.6f}\n")
                    total += 1
            print(f"🎉 성공: 총 {total}개 좌표 저장 완료!")
except Exception as e:
    print(f"❌ 에러 발생: {e}")