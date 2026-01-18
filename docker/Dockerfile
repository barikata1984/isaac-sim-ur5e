FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root

# Antigravity リモートサーバーのダウンロードに必要
RUN apt-get update \
    && apt-get install -y git wget \
    && rm -rf /var/lib/apt/lists/*

# 標準的なコマンドが確実に使えるようにパスを設定
ENV PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

# python.sh へのシンボリックリンクを作成
# これにより、どこからでも "python" と打つだけで Isaac Sim 環境が立ち上がります
RUN ln -s /isaac-sim/python.sh /usr/local/bin/python

# ログイン時に自動でワークスペースに移動するように設定
RUN echo "cd /workspaces/isaac-sim-ur5e" >> /root/.bashrc

USER root
