FROM node:22-alpine AS build

WORKDIR /app
ENV NPM_CONFIG_CACHE=/app/.npm-cache

COPY web/package.json web/package-lock.json ./
RUN npm ci --no-audit --no-fund

COPY web/ ./
RUN npm run build

FROM node:22-alpine AS runtime

WORKDIR /app
ENV NPM_CONFIG_CACHE=/app/.npm-cache
RUN npm install --global http-server@14.1.1 --no-audit --no-fund

COPY --from=build /app/dist/ /opt/web-dist/

RUN mkdir -p /output
EXPOSE 8080

CMD ["sh", "-c", "cp -R /opt/web-dist/. /output/ && exec http-server /opt/web-dist -a 0.0.0.0 -p 8080 -c-1"]
