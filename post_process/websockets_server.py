import asyncio
import websockets
import json

# 处理客户端连接
async def handle_client(websocket, path):
    async for message in websocket:
        try:
            # 解析接收到的 JSON 数据
            data = json.loads(message)
            print("Received JSON:", data)

            # 可以在这里对 JSON 数据进行处理
            response = {"status": "success", "received": data}

            # 返回响应给客户端
            await websocket.send(json.dumps(response))
        except json.JSONDecodeError:
            # 如果数据不是有效的 JSON 格式
            error_response = {"status": "error", "message": "Invalid JSON"}
            await websocket.send(json.dumps(error_response))

# 启动 WebSocket 服务器
start_server = websockets.serve(handle_client, "localhost", 8765)

print("WebSocket server started on ws://localhost:8765")
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()