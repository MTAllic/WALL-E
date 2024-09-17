#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # You can adjust the message type as needed
import requests

class LLMPublisher(Node):
    def __init__(self):
        super().__init__('llm_publisher')
        self.subscription = self.create_subscription(
            String,
            '/example_topic',  # Replace with your desired ROS topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received data: "%s"' % msg.data)
        self.send_to_llm(msg.data)

    def send_to_llm(self, data):
        # Format the data for LLM request
        formatted_data = f"Received the following data from ROS2: {data}"

        # Send the data to GPT-4 API
        response = self.send_to_gpt4(formatted_data)
        self.get_logger().info('LLM Response: "%s"' % response)

    def send_to_gpt4(self, data):
        # Replace with your OpenAI API key
        api_key = "your_openai_api_key_here"
        
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {api_key}',
        }

        # Payload for the GPT-4 API
        payload = {
            "model": "gpt-4",
            "messages": [{"role": "system", "content": "You are a helpful assistant."},
                         {"role": "user", "content": data}],
            "max_tokens": 150
        }

        # Send the data to GPT-4 via POST request
        response = requests.post("https://api.openai.com/v1/chat/completions", json=payload, headers=headers)
        
        if response.status_code == 200:
            return response.json()['choices'][0]['message']['content']
        else:
            return f"Error: {response.status_code} - {response.text}"

def main(args=None):
    rclpy.init(args=args)
    llm_publisher = LLMPublisher()
    rclpy.spin(llm_publisher)
    llm_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

