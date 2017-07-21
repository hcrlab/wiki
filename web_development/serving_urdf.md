To serve a URDF of a robot in a way that works with [ros3djs](http://wiki.ros.org/ros3djs/Tutorials/VisualizingAURDF), copy the `ROBOT_description` package:

```
mkdir ~/local/collada
cp -r /opt/ros/indigo/share/pr2_description ~/local/collada
```

You need to serve it as static content, but you must explicitly allow cross-origin requests.
One way to do this is to use [Caddy](https://caddyserver.com/)
- Click download, but before downloading, add the "http.cors" plugin.
- Install caddy. Typically this just means untarring it and adding it to your PATH.
- Add the following Caddyfile to ~/local/collada:

```
cd ~/local/collada
vim Caddyfile
```

**Caddyfile**
```
localhost:8001
cors
```

Now to serve the URDF, just run Caddy:
```
cd ~/local/collada
caddy
```

## In production
On a real robot, you can do the same thing, but serve the files using Apache.
As before, copy the ~/local/collada folder to /var/ros/collada.

Then, add the following Apache configuration:
```
sudo vim /etc/apache2/sites-enabled/collada.conf
```

```ApacheConf
Listen 8001
<VirtualHost *:8001>
  ServerAdmin robot@university.edu
  ServerName robot.university.edu
  ServerAlias robot

  DocumentRoot /var/ros/collada
  <Directory />
    Options FollowSymLinks
    AllowOverride None
    Order Deny,Allow
    Deny from All
  </Directory>
  <Directory /var/ros/collada>
    Header set Access-Control-Allow-Origin "*"
    Options Indexes FollowSymLinks MultiViews
    AllowOverride None
    Order allow,deny
    allow from all
    require all granted
  </Directory>

  ErrorLog ${APACHE_LOG_DIR}/error.log

  # Possible values include: debug, info, notice, warn, error, crit,
  # alert, emerg.
  LogLevel warn

  CustomLog ${APACHE_LOG_DIR}/access.log combined
</VirtualHost>
```

Enable the "headers" module:
```
sudo a2enmod headers
```

Then, enable your site:
```
sudo a2ensite collada
```

The robot should now be serving the URDF files properly.
