FROM node:22-alpine AS build

WORKDIR /app
ENV NPM_CONFIG_CACHE=/app/.npm-cache

COPY web/package.json web/package-lock.json ./
RUN npm ci --no-audit --no-fund

COPY web/ ./
RUN npm run build

FROM alpine:3.22 AS export

COPY --from=build /app/dist/ /opt/web-dist/

CMD ["cp", "-R", "/opt/web-dist/.", "/output/"]
