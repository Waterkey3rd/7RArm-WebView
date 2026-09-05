FROM node:22-alpine AS build

WORKDIR /app
ENV NPM_CONFIG_CACHE=/app/.npm-cache

COPY web/package.json web/package-lock.json ./
RUN npm ci --no-audit --no-fund

COPY web/ ./
RUN npm run build

FROM nginx:1.28-alpine AS runtime

COPY --from=build /app/dist/ /usr/share/nginx/html/

EXPOSE 80
