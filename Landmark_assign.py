import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches




# 获取平行四边形在x轴上的范围
def get_x_range(vertices):
    global x_min, x_max  # Declare x_min and x_max as global
    x_values = [x for x, _ in vertices]
    x_min = min(x_values)
    x_max = max(x_values) 

def is_rhombus_above_bottom(rect, rhombus_vertices):
    _, rect_bottom_y = rect[0]
    top_y = max(y for _, y in rhombus_vertices)
    return top_y >= rect_bottom_y

# 平移等边平行四边形的顶点
def translate(vertices, dx, dy):
    return [(x + dx, y + dy) for x, y in vertices]

# 计算等边平行四边形的顶点
def calculate_rhombus_vertices(A, l):
    x1, y1 = A
    x1 = x1 - l - l / 3
    A = (x1,y1)
    E = (x1 + l, y1)
    F = (x1 + l / 2, y1 - l * math.sqrt(3) / 2)
    G = (x1 + l + l / 2, y1 - l * math.sqrt(3) / 2)
    return [A, E, F, G]

# 判断landmarks是否在长方形内
def is_landmark_inside_rectangle(landmark, rect_A, rect_C):
    rect_left_x, rect_bottom_y = rect_A
    rect_right_x, rect_top_y = rect_C
    x, y = landmark
    return rect_left_x <= x <= rect_right_x and rect_bottom_y <= y <= rect_top_y

def insert_landmarks(vertices, l, rectangle_A, rectangle_C,rhombus_id):
    
    codenames = ['0000', '0001', '0011', '0110', '0010', '0100', '0111', '1010', '0101', '1000', '1011', '1101', '1001', '1100', '1110', '1111']
    # 初始化landmarks集合
    landmarks = []

    # 获取等边平行四边形的边长向量
    AB = (vertices[1][0] - vertices[0][0], vertices[1][1] - vertices[0][1])
    AD = (vertices[2][0] - vertices[0][0], vertices[2][1] - vertices[0][1])

    # 将边长向量分割为3份，得到小三角形的边长向量
    AB_small = (AB[0] / 3, AB[1] / 3)
    AD_small = (AD[0] / 3, AD[1] / 3)

    # 生成网格中的landmarks
    for i in range(4):
        for j in range(4):
            # 计算每个小三角形的顶点
            landmark_x = vertices[0][0] + j * AB_small[0] + i * AD_small[0]
            landmark_y = vertices[0][1] + j * AB_small[1] + i * AD_small[1]
            #landmarks.append((landmark_x, landmark_y))
            codename = codenames[i * 4 + j]
            landmarks.append(((landmark_x, landmark_y), codename))
    
    inside_landmarks_with_codenames = [(lm, code) for (lm, code) in landmarks if is_landmark_inside_rectangle(lm, rectangle_A, rectangle_C)]
    #inside_landmarks = [lm for lm in landmarks[:16] if is_landmark_inside_rectangle(lm, rectangle_A, rectangle_C)]
    
    
    
    return inside_landmarks_with_codenames


# 主程序开始
# 设置长方形和等边平行四边形的参数

#Coordinates of the lower left Corner
rectangle_A = (-3885, -10020) 
#Coordinates of the upper right Corner
rectangle_C = (4255, 10020)  
 






rhombus_vertex = (-3885, 10020)  # 等边平行四边形左上角顶点坐标

l = 6000  # 边长
translation_distance = l + l / 3

# 计算长方形对角线顶点坐标
rectangle = [rectangle_A, rectangle_C]

# 初始化等边平行四边形顶点坐标
rhombus_vertices = calculate_rhombus_vertices(rhombus_vertex, l)


# 设置初始方向为向右
move_right = True
rhombus_set = set()
rhombus_set.add(tuple(map(tuple, rhombus_vertices)))
get_x_range(rhombus_vertices)

rect_left_x, _ = rectangle_A
rect_right_x, _ = rectangle_C


while True:


    # 检查平行四边形是否需要改变方向
    if move_right and x_min <= rect_right_x:
        # 如果向右移动并且平行四边形仍在长方形内，继续向右移动
        dx = translation_distance
        dy = 0
    elif not move_right and x_max >= rect_left_x:
        # 如果向左移动并且平行四边形仍在长方形内，继续向左移动
        dx = -translation_distance
        dy = 0
    else:
        # 如果平行四边形到达边界，改变方向并斜向下移动
        dx = translation_distance / 2
        dy = -translation_distance * math.sin(math.radians(60))
        move_right = not move_right  # 改变水平移动方向
        
    # 移动平行四边形
    rhombus_vertices = translate(rhombus_vertices, dx, dy)
    get_x_range(rhombus_vertices)
    
    # 检查平行四边形是否完全低于长方形底部
    if not is_rhombus_above_bottom(rectangle, rhombus_vertices):
        
        break  # 如果低于，则退出循环
    
    
        
    if((x_max >= rect_left_x and x_max <= rect_right_x) or (x_min >= rect_left_x and x_min <= rect_right_x)):
        rhombus_set.add(tuple(map(tuple, rhombus_vertices)))
    
'''print("Rhombus positions within the rectangle:")
for rhombus in rhombus_set:
    print(rhombus)'''
    
# Function to extract the sorting key from a rhombus
def rhombus_sort_key(rhombus):
    # Get the smallest x-coordinate and the largest y-coordinate of the rhombus
    min_x = min(x for x, _ in rhombus)
    max_y = max(y for _, y in rhombus)
    # Return a tuple that will be used for sorting (smallest x, then largest y)
    return (min_x, -max_y)


all_landmarks = []
rhombus_id = 0
 # Create a dictionary that maps rhombus identifier to list of codenames of kept landmarks
rhombus_landmarks_dict = {}
for rhombus in rhombus_set:
    
    # 计算并插入landmarks
    landmarks = insert_landmarks(rhombus, l,rectangle_A, rectangle_C,rhombus_id)
     # Create a dictionary that maps rhombus identifier to list of codenames of kept landmarks
     # Update the dictionary with the rhombus identifier and the associated tuples of kept landmarks and codenames
    rhombus_landmarks_dict[rhombus_id] = landmarks
    
    # Increment the rhombus identifier for the next iteration
    rhombus_id += 1
    
print(rhombus_landmarks_dict)
# 绘制所有的菱形和landmarks
fig, ax = plt.subplots()

# 绘制长方形边界
rectangle_width = rectangle_C[0] - rectangle_A[0]
rectangle_height = rectangle_C[1] - rectangle_A[1]
rectangle_patch = patches.Rectangle(rectangle_A, rectangle_width, rectangle_height, linewidth=1, edgecolor='black', facecolor='none')
ax.add_patch(rectangle_patch)

for rhombus in rhombus_set:
    # 绘制菱形
    rhombus_patch = patches.Polygon(rhombus, closed=True, edgecolor='blue', facecolor='none')
    ax.add_patch(rhombus_patch)

# 绘制并标注landmarks
for rhombus_id, landmarks in rhombus_landmarks_dict.items():
    for (landmark, codename) in landmarks:
        ax.plot(*landmark, 'ro')  # 绘制landmark
        ax.text(landmark[0], landmark[1], str(codename), color='green', fontsize=8)  # 标注codename

# 设置图形的边界
ax.set_xlim(rectangle_A[0] - 1000, rectangle_C[0] + 1000)
ax.set_ylim(rectangle_A[1] - 1000, rectangle_C[1] + 1000)

# 显示图形
plt.grid(True)
plt.show()


# Open a file named 'landmarks_results.txt' in write mode
'''with open('landmarks_results.txt', 'w', encoding='utf-8') as file:
    # Write a header line to the file
    file.write("Rhombus ID: (Landmark Coordinates), Codename\n")
    file.write("-------------------------------------------------\n")
    
    # Iterate over the rhombus_landmarks_dict to get each rhombus and its landmarks
    for rhombus_id, landmarks in rhombus_landmarks_dict.items():
        file.write(f"Rhombus {rhombus_id}:\n")
        # Iterate over the landmarks for the current rhombus
        for landmark, codename in landmarks:
            # Write the landmark coordinates and codename to the file
            file.write(f"    ({landmark[0]}, {landmark[1]}), Codename: {codename}\n")
        file.write("\n")  # Add a newline for formatting'''

# The file will automatically be closed when exiting the 'with' block