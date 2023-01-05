# localization_launch

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

You can include as follows in `*.launch.xml` to use `localization.launch.xml`.

```xml
  <include file="$(find-pkg-share localization_launch)/launch/localization.launch.xml">
  </include>
```

## Notes

There are some `param.yaml` files in `config` directory.

```bash
ndt_scan_matcher.param.yaml
```
