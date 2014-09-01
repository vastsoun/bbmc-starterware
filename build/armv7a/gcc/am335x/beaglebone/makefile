# Locating the root directory 
#
ROOT=../../../../../

#
# Target Directories that need to be built
#
DIRS= platform bootloader

#
# Building each application
#
all:
	@for i in ${DIRS};                                           \
	do                                                           \
		if [ -f $${i}/makefile ] ;                 \
		then                                                 \
			make -C $${i}/ || exit $$?;        \
		fi;                                                  \
	done

#
# Clean
#
clean:
	@for i in ${DIRS};                                           \
	do                                                           \
		if [ -f $${i}/makefile ] ;                 \
		then                                                 \
			make -C $${i}/ clean || exit $$?;  \
		fi;                                                  \
	done

clean+:
	@for i in ${DIRS};                                           \
	do                                                           \
		if [ -f $${i}/makefile ] ;                 \
		then                                                 \
			make -C $${i}/ clean+ || exit $$?;  \
		fi;                                                  \
	done
