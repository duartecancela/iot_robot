// src/db.js

import { MongoClient } from "mongodb";

const MONGO_URL = process.env.MONGO_URL || "mongodb://127.0.0.1:27017";
const MONGO_DB_NAME = process.env.MONGO_DB_NAME || "iot_robot";

let client = null;
let db = null;

export async function connectMongo() {
  client = new MongoClient(MONGO_URL);

  await client.connect();

  db = client.db(MONGO_DB_NAME);

  console.log("MongoDB connected:", MONGO_URL);
  console.log("MongoDB database:", MONGO_DB_NAME);

  return db;
}

export function getMongoDb() {
  if (!db) {
    throw new Error("MongoDB is not connected");
  }

  return db;
}

export async function closeMongo() {
  if (client) {
    await client.close();
    client = null;
    db = null;
  }
}