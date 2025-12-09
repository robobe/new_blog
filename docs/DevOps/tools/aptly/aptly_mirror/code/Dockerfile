FROM ubuntu:24.04

COPY aptly.asc /etc/apt/keyrings/aptly.asc
COPY sources.list /etc/apt/sources.list
RUN chmod 644 /etc/apt/keyrings/aptly.asc \
    && rm -rf /etc/apt/sources.list.d

CMD ["/bin/bash"]