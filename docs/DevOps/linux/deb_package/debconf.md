---
tags:
    - debconf
    - debian
    - linux
    - package
---

# Using Debconf

Debconf is Debian’s configuration management system. It stores user inputs and configuration options for packages during installation, upgrade, and removal.

## Structure of the templates File

A templates file consists of multiple template entries, each defining a specific question. Each entry follows this format:

```
Template: mypackage/question_name
Type: string
Default: default_value
Description: A brief explanation of what the user should enter.
```

|Field |	Description |
| ---- | -------------  |
|Template | Unique identifier for the question (mypackage/question_name). |
|Type | The type of input expected (see below for options). |
|Default |	The default answer if the user does not provide one. |
|Description |	The message displayed to the user. |

### Types

- **string**:	User enters a free-text string.
- **boolean**:	Yes/No question.
- **select**:	User selects from predefined options.
- **multiselect**:	User selects multiple options.
- **note**:	Displays a message but does not ask a question.
- **password**:	Hides user input for security.


```title="Boolean (Yes/No)"
Template: mypackage/enable_feature
Type: boolean
Default: true
Description: Do you want to enable this feature?
```


``` title="Select (Multiple Choices)"
Template: mypackage/choose_option
Type: select
Choices: Option1, Option2, Option3
Default: Option1
Description: Please choose an option.
```

``` title="Multiselect (Multiple Selections)"
Template: mypackage/multiple_choices
Type: multiselect
Choices: Red, Green, Blue
Default: Red, Blue
Description: Select colors (use spacebar to select multiple).
```

```title="Note (Just a Message, No Input)"
Template: mypackage/info_message
Type: note
Description: This package will install additional dependencies.
```


## Read the User's Input in postinst

Once the user enters values during installation, you can retrieve them in debian/postinst:

```bash title="postinst"
#!/bin/bash
set -e
. /usr/share/debconf/confmodule

db_input high mypackage/config_value || true
db_go

db_get mypackage/config_value
echo "User entered: $RET" > /etc/mypackage/config.txt
```

Tells Debconf to show a prompt for mypackage/config_value.
high is the priority (other options: low, medium, critical).
|| true ensures the script does not fail if db_input encounters an issue.


Displays all pending Debconf questions in one go.
Must be called after db_input to show the prompt to the user.

Retrieves the user's input from Debconf.
Stores the value in the $RET variable.

## postrm
```bash title="postrm"
#!/bin/sh
set -e  # Exit on any error

# Example: Remove the config directory on purge
if [ "$1" = "purge" ]; then
    echo "Configuration directory removed."
    echo "PURGE" | debconf-communicate my-python-app || true
fi


exit 0
```

## other debconf commands
6. Debugging and Testing

To check stored values:

```
sudo debconf-show mypackage
```

To manually set a value:
```
echo "mypackage/config_value mypackage/config_value string new_value" | sudo debconf-set-selections
```

To reset and ask the questions again:
```
sudo dpkg-reconfigure mypackage
```



!!! tip "package script file location"
    ```bash
    ls -l /var/lib/dpkg/info/mypackage.*
    #
    This will show various control files for the package, including:

    /var/lib/dpkg/info/mypackage.postrm → The post-remove script
    /var/lib/dpkg/info/mypackage.postinst → The post-install script
    /var/lib/dpkg/info/mypackage.prerm → The pre-remove script
    /var/lib/dpkg/info/mypackage.list → The list of installed files
    ```