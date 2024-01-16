.PHONEY: build-firmware

build-firmware:
	. /opt/ros/noetic/setup.bash
	# cw
	# roscd linorobot/teensy/firmware
	# platformio run --target upload -e teensy41

# apply-dns add-user add-user-files apply-deployment delete-user delete-user-files env

# apply-dns:
# 	terraform apply \
# 		-auto-approve \
# 		-var "ingress_ip=${INGRESS_IP}" \
# 		-var "aws_access_key=${AWS_ACCESS_KEY}" \
# 		-var "aws_secret_key=${AWS_SECRET_KEY}"

# add-user: add-user-files apply-dns apply-deployment

# add-user-files:
# 	cp -r clouddesktop-storage-template $(id)-clouddesktop-storage
# 	cp -r clouddesktop-template $(id)-clouddesktop
# 	cp clouddesktop-template.tf.tmpl $(id)-clouddesktop.tf
# 	find $(id)-clouddesktop-storage -type f -exec sed -i "" 's/CD_USER/$(id)/g' {} \;
# 	find $(id)-clouddesktop -type f -exec sed -i "" 's/CD_USER/$(id)/g' {} \;
# 	sed -i "" 's/CD_USER/$(id)/g' $(id)-clouddesktop.tf;

# apply-deployment:
# 	kubectl apply -k $(id)-clouddesktop-storage
# 	kubectl apply -k $(id)-clouddesktop

# delete-user: delete-deployments delete-user-files

# delete-deployments:
# 	kubectl delete -k $(id)-clouddesktop
# 	kubectl delete -k $(id)-clouddesktop-storage

# delete-user-files:
# 	rm -rf $(id)-clouddesktop
# 	rm -rf $(id)-clouddesktop-storage
# 	rm $(id)-clouddesktop.tf

# instructions:
# 	@sed 's/CD_USER/$(id)/g' instructions.md
