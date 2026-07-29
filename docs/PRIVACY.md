# Privacy

VibeStick Hourglass Liquid runs locally on the StickS3. The firmware does not
connect to Wi-Fi, contact remote services, record audio, or collect user data.

## Publishing checklist

Before publishing source code or a firmware image:

- use a GitHub `noreply` email for commits when personal email disclosure is
  not intended;
- remove local absolute paths, serial device identifiers, credentials, logs,
  recordings, and build caches;
- inspect compiled binaries for embedded local paths, email addresses, and
  credentials;
- strip unnecessary image metadata; and
- review every branch, tag, and pull-request ref that will remain reachable.

The checked-in application image was inspected for personal paths, email
addresses, and credential-like strings before publication.
