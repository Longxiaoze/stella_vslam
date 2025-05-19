FROM node:8.16.0-alpine

COPY . /stella_vslam-viewer/

RUN set -x && \
  cd /stella_vslam-viewer/ && \
  npm install

ENTRYPOINT ["node", "/stella_vslam-viewer/app.js"]
