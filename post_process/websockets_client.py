import asyncio
import websockets
import json

# 要发送的 JSON 数据
json_data = {
    "name": "example",
    "age": 25,
    "location": "Earth"
}

# 发送 JSON 数据到服务器
async def send_json():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        # 将 JSON 数据转换为字符串并发送
        await websocket.send(json.dumps(json_data))
        print("Sent JSON:", json_data)

        # 接收服务器的响应
        response = await websocket.recv()
        print("Received from server:", response)

# 运行客户端
asyncio.get_event_loop().run_until_complete(send_json())