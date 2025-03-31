---
tags:
    - gpg
    - encrypt
---

# GPG

- **GPG (GNU Privacy Guard)** is an open-source implementation of the OpenPGP standard used for encryption, signing, and verifying data
- **GPG2 (GNU Privacy Guard 2)** is an updated version of GPG (GnuPG) that provides improved cryptographic features, better security, and supports modern encryption algorithms.



```bash title="encrypt"
gpg --symmetric --cipher-algo AES256 --passphrase-file password.txt --batch secret.txt
```

```bash title="decrypt"
gpg --decrypt --passphrase-file password.txt --batch secret.txt.gpg > secret.txt

```

- --batch: Prevent GPG from prompting for interactive input.