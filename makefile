all: install start enable

clean: disable stop uninstall

install:
	ln -sf $(CURDIR)/systemd/enviroplus_exporter.service /etc/systemd/system/enviroplus_exporter.service

uninstall:
	rm -f /etc/systemd/system/enviroplus_exporter.service

start:
	systemctl start enviroplus_exporter

stop:
	systemctl stop enviroplus_exporter

enable:
	systemctl enable enviroplus_exporter

disable:
	systemctl disable enviroplus_exporter
