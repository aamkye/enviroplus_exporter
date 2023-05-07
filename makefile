all: preinstall install start enable

clean: disable stop uninstall

preinstall:
	pip install -R requirements.txt

install:
	cp $(CURDIR)/enviroplus_exporter.py /usr/bin/enviroplus_exporter.py
	ln -sf $(CURDIR)/systemd/enviroplus_exporter.service /etc/systemd/system/enviroplus_exporter.service
	systemctl daemon-reload

uninstall:
	rm -f /etc/systemd/system/enviroplus_exporter.service
	rm -f /usr/bin/enviroplus_exporter.py
	systemctl daemon-reload

start:
	systemctl start enviroplus_exporter

stop:
	systemctl stop enviroplus_exporter

enable:
	systemctl enable enviroplus_exporter

disable:
	systemctl disable enviroplus_exporter

upgrade:
	git pull && make install && systemctl restart enviroplus_exporter
