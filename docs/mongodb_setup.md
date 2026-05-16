# MongoDB Setup (Raspberry Pi 4 + Docker)

## System Information

Tested on:

- Raspberry Pi 4
- Debian 12 (Bookworm)
- ARM64 (`aarch64`)

Check architecture:

```bash
uname -m
```

Check operating system:

```bash
cat /etc/os-release
```

---

# Docker Installation

Update package list:

```bash
sudo apt update
```

Install Docker:

```bash
sudo apt install -y docker.io
```

Enable Docker service:

```bash
sudo systemctl enable docker
sudo systemctl start docker
```

Check Docker version:

```bash
docker --version
```

Check Docker service status:

```bash
sudo systemctl status docker
```

Test Docker installation:

```bash
sudo docker run hello-world
```

---

# MongoDB Container

## Important Note

The latest MongoDB versions may not work correctly on Raspberry Pi 4 because they require ARMv8.2-A instructions.

The following version was tested successfully:

```text
mongo:4.4.18
```

---

# Create MongoDB Container

Run MongoDB container:

```bash
sudo docker run -d \
  --name robot-mongo \
  -p 27017:27017 \
  -v robot_mongo_data:/data/db \
  mongo:4.4.18
```

---

# Verify Container Status

Check running containers:

```bash
sudo docker ps
```

Expected result:

```text
STATUS Up
```

---

# View MongoDB Logs

```bash
sudo docker logs robot-mongo
```

Expected message:

```text
Waiting for connections
```

---

# MongoDB Connection

Default MongoDB port:

```text
27017
```

Get Raspberry Pi IP address:

```bash
hostname -I
```

MongoDB connection string:

```text
mongodb://iotrobot.local:27017
```

or:

```text
mongodb://RASPBERRY_PI_IP:27017
```

---

# Useful Docker Commands

Stop container:

```bash
sudo docker stop robot-mongo
```

Start container:

```bash
sudo docker start robot-mongo
```

Restart container:

```bash
sudo docker restart robot-mongo
```

Remove container:

```bash
sudo docker rm -f robot-mongo
```

View logs:

```bash
sudo docker logs robot-mongo
```

List containers:

```bash
sudo docker ps
```

List all containers:

```bash
sudo docker ps -a
```

---

# Persistent Storage

MongoDB data is stored in the Docker volume:

```text
robot_mongo_data
```

This ensures the database persists even if the container is restarted.

List Docker volumes:

```bash
sudo docker volume ls
```