FROM node:24.20.0-bookworm-slim AS build
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci
COPY . .
RUN npm run build

FROM node:24.20.0-bookworm-slim
WORKDIR /app
COPY --from=build /app/dist/client ./dist/client
COPY scripts ./scripts
COPY lib/home.ts ./lib/home.ts
RUN mkdir /app/data && chown node:node /app/data
USER node
ENV HOMEBASE_LISTEN_HOST=0.0.0.0
ENV HOMEBASE_DATA_DIR=/app/data
EXPOSE 5173
CMD ["node", "scripts/serve.mjs"]
