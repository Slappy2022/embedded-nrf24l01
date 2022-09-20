default:
	find . |grep -v /target |grep -v "/\." | entr -ds \
		'cargo test'
build:
	cargo build
test:
	cargo test
clean:
	cargo clean
