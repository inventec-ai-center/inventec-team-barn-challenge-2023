# Installing SingularityCE

[Reference Link](https://github.com/sylabs/singularity/blob/main/INSTALL.md?plain=1)

Since you are reading this from the SingularityCE source code, it will be
assumed that you are building/compiling from source.

For full instructions on installation, including building RPMs, please check the
[installation section of the admin guide](https://sylabs.io/guides/latest/admin-guide/).

## Install system dependencies

You must first install development tools and libraries to your host.

On Debian-based systems, including Ubuntu 20.04 and above:

```sh
# Ensure repositories are up-to-date
sudo apt-get update
# Install debian packages for dependencies
sudo apt-get install -y \
    build-essential \
    libseccomp-dev \
    libglib2.0-dev \
    pkg-config \
    squashfs-tools \
    cryptsetup \
    crun \
    uidmap \
    git \
    wget
```

_Note - `crun` / `runc` can be omitted if you will not use the `singularity oci`
commands, or the `--oci` execution mode._

## Install Go

Singularity is written in Go, and may require a newer version of Go than is
available in the repositories of your distribution. We recommend installing the
latest version of Go from the [official binaries](https://golang.org/dl/).

First, download the Go tar.gz archive to `/tmp`, then extract the archive to
`/usr/local`.

_**NOTE:** if you are updating Go from a older version, make sure you remove
`/usr/local/go` before reinstalling it._

```sh
export VERSION=1.20.5 OS=linux ARCH=amd64  # change this as you need

wget -O /tmp/go${VERSION}.${OS}-${ARCH}.tar.gz \
  https://dl.google.com/go/go${VERSION}.${OS}-${ARCH}.tar.gz
sudo tar -C /usr/local -xzf /tmp/go${VERSION}.${OS}-${ARCH}.tar.gz
```

Finally, add `/usr/local/go/bin` to the `PATH` environment variable:

```sh
echo 'export PATH=$PATH:/usr/local/go/bin' >> ~/.bashrc
source ~/.bashrc
```

## Install golangci-lint

If you will be making changes to the source code, and submitting PRs, you should
install `golangci-lint`, which is the linting tool used in the SingularityCE
project to ensure code consistency.

Every pull request must pass the `golangci-lint` checks, and these will be run
automatically before attempting to merge the code. If you are modifying
Singularity and contributing your changes to the repository, it's faster to run
these checks locally before uploading your pull request.

In order to download and install the latest version of `golangci-lint`, you can
run:

<!-- markdownlint-disable MD013 -->

```sh
curl -sSfL https://raw.githubusercontent.com/golangci/golangci-lint/master/install.sh | sh -s -- -b $(go env GOPATH)/bin
```

<!-- markdownlint-enable MD013 -->

Add `$(go env GOPATH)` to the `PATH` environment variable:

```sh
echo 'export PATH=$PATH:$(go env GOPATH)/bin' >> ~/.bashrc
source ~/.bashrc
```

## Clone the repo

With the adoption of Go modules you no longer need to clone the SingularityCE
repository to a specific location.

Clone the repository with `git` in a location of your choice:

```sh
git clone --recurse-submodules https://github.com/sylabs/singularity.git
cd singularity
```

By default your clone will be on the `main` branch which is where development
of SingularityCE happens. To build a specific version of SingularityCE, check
out a [release tag](https://github.com/sylabs/singularity/tags) before
compiling. E.g. to build the 3.11.4 release, checkout the
`v3.11.4` tag:

```sh
git checkout --recurse-submodules v3.11.4
```

## Compiling SingularityCE

You can configure, build, and install SingularityCE using the following
commands:

```sh
./mconfig
make -C builddir
sudo make -C builddir install
```

And that's it! Now you can check your SingularityCE version by running:

```sh
singularity --version
```

The `mconfig` command accepts options that can modify the build and installation
of SingularityCE. For example, to build in a different folder and to set the
install prefix to a different path:

```sh
./mconfig -b ./buildtree -p /usr/local
```

See the output of `./mconfig -h` for available options.


For more information on installing/updating/uninstalling the RPM, check out our
[admin docs](https://www.sylabs.io/guides/latest/admin-guide/admin_quickstart.html).