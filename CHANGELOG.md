# CHANGELOG


## v5.0.1-rc.3 (2025-04-09)

### Bug Fixes

- Add typing stubs for defusedxml, colorama, PyYAML, and tqdm to pyproject.toml
  ([`536bd4c`](https://github.com/MGross21/mujoco-toolbox/commit/536bd4cb7ea34edd17c87012379bb19004dc3772))

- Correct attribute name for initial conditions in Wrapper class
  ([`ec03d88`](https://github.com/MGross21/mujoco-toolbox/commit/ec03d88c583a4fe7d529bbeae4a8f1d04e92b617))

- Enhance initial_conditions setter validation and streamline attribute assignment
  ([`d452acf`](https://github.com/MGross21/mujoco-toolbox/commit/d452acfdd69b387acb2de2f75bab119b3c9f4576))

- Enhance XML loading and error handling in Loader class
  ([`8842d8c`](https://github.com/MGross21/mujoco-toolbox/commit/8842d8c53c5641eb2eb3d2e0792c392b89c9f1b5))

- Ensure proper closure of conditional statement in CI workflow
  ([`799f234`](https://github.com/MGross21/mujoco-toolbox/commit/799f234e4d2b713ead71ac8d4dc1849d1c0e7a28))

- Iinstallation script and mypy configuration for MuJoCo stubs
  ([`ef2055b`](https://github.com/MGross21/mujoco-toolbox/commit/ef2055b0c88248c77ab57643eba40aabd479c678))

- Improve data_rate setter validation in Wrapper class
  ([`8c183d5`](https://github.com/MGross21/mujoco-toolbox/commit/8c183d5cecb7dfa586cd4115f49172e9963d4774))

- Improve docstring formatting for clarity in Wrapper class methods
  ([`0a2124f`](https://github.com/MGross21/mujoco-toolbox/commit/0a2124f26fcce0ee6b2833f53de81920e8c15534))

- Optimize simulation step handling in Wrapper class (Ghosted #20)
  ([`3a95ac5`](https://github.com/MGross21/mujoco-toolbox/commit/3a95ac572615e426550c06604479c6045969c33e))

- Replace xml.etree with defusedxml for safer XML parsing and update resolution extraction logic
  ([`baf0aff`](https://github.com/MGross21/mujoco-toolbox/commit/baf0aff14eed90775914236d3519cc0a6a9d9cdb))

- Update .gitignore to include CMake and _deps directories
  ([`59b3e47`](https://github.com/MGross21/mujoco-toolbox/commit/59b3e475245eb2d4d89ee60d24497325682916ce))

- Update data structure for CAPTURE_PARAMETERS and improve key validation in Wrapper class
  ([`a5aaeee`](https://github.com/MGross21/mujoco-toolbox/commit/a5aaeeec5f47c19720f7cdc64ce0302353ff287a))

- Update dependencies in pyproject.toml and add coverage tools; remove commented-out mypy and
  id2name tests
  ([`ccaa9b3`](https://github.com/MGross21/mujoco-toolbox/commit/ccaa9b319c56360168ff16936fb8659be6a782ab))

- Update development requirements and Python version constraints
  ([`44982be`](https://github.com/MGross21/mujoco-toolbox/commit/44982bec6d62a7e9b00c7a55a31befbd9a79d984))

- Update linter commands to include unsafe fixes and remove checks for black and isort
  ([`969b0ad`](https://github.com/MGross21/mujoco-toolbox/commit/969b0ad16503157af6fada39b47417c4089f5745))

- Update MuJoCo bindings and installation process; add mypy to dev requirements
  ([`880bdfc`](https://github.com/MGross21/mujoco-toolbox/commit/880bdfc8a5f1e3e02c883c27a942e41fe5075297))

- Update mypy and typing stubs to use wildcard versions for better compatibility
  ([`765025e`](https://github.com/MGross21/mujoco-toolbox/commit/765025edf9909f4eeac235ea3b6eed82db2c4312))

- Update name2id tests to assert ValueError for nonexistent IDs
  ([`a3fd992`](https://github.com/MGross21/mujoco-toolbox/commit/a3fd99203979a509bc0b0e70a8f936953317c835))

- Update pytest version to ^8.3.5 in pyproject.toml
  ([`5a0fed1`](https://github.com/MGross21/mujoco-toolbox/commit/5a0fed1ceae3a02fafd020f200b0bb14da246bce))

- Update test_wrapper.py to improve model path handling and add validation tests
  ([`946133c`](https://github.com/MGross21/mujoco-toolbox/commit/946133ce72f05e871937a7d92d867d7e6a879fee))

- Update Wrapper class initialization docstring for clarity and add error handling
  ([`0831331`](https://github.com/MGross21/mujoco-toolbox/commit/0831331e46200d00f99dc0e4ef5bc59cdd7c6db8))

### Refactoring

- Comment out deprecated URDF test function and improve parameter naming
  ([`59110ef`](https://github.com/MGross21/mujoco-toolbox/commit/59110ef706f1c5a0bedc2dd71379a7278d5641da))

- Remove deprecated ur5e_glovebox example script
  ([`a64cd38`](https://github.com/MGross21/mujoco-toolbox/commit/a64cd38ae1fb740c697e24ebf4aa3bf50cc10371))

- Remove render bypass method
  ([`d618b84`](https://github.com/MGross21/mujoco-toolbox/commit/d618b8426086925b3ade87e77e7d0ccc5b927bb2))


## v5.0.1-rc.2 (2025-04-08)

### Bug Fixes

- Clean up imports and add TODO comment for future implementation in ur5e_glovebox example
  ([`48d8db3`](https://github.com/MGross21/mujoco-toolbox/commit/48d8db38b85559d86508da1ec7e6feb0e47ec7d1))

- Clear console output when creating a new Wrapper instance
  ([`5f8505f`](https://github.com/MGross21/mujoco-toolbox/commit/5f8505f4c5955df188259c1594e77e786ae6eae3))

- Enhance MuJoCo setup script with download and extraction functionality
  ([`69d40ec`](https://github.com/MGross21/mujoco-toolbox/commit/69d40eca0c39ebbb37b4267f21c07ad84008823b))

- Improve error handling in _skip_rendering and enhance video title management
  ([`996ecdd`](https://github.com/MGross21/mujoco-toolbox/commit/996ecdd5eac359d4c3869ad7260096b30bf1dcfa))

- Move world assets to separate XML file and update asset loading in assets.py
  ([`5ae5656`](https://github.com/MGross21/mujoco-toolbox/commit/5ae56561fb7ccaf50ce1bd55fb6b892801d4ae7f))

- Occlude Builder from __all__
  ([`ede0a6d`](https://github.com/MGross21/mujoco-toolbox/commit/ede0a6dbb577d096422787c9d2b8178e12d230cf))

- Re-add progress bar support in Wrapper class and update dependencies in pyproject.toml
  ([`ee80f58`](https://github.com/MGross21/mujoco-toolbox/commit/ee80f581f077eec0f98b2925d744abebe33f85f8))

- Refactor _load_model method for improved file extension handling and XML validation
  ([`187402b`](https://github.com/MGross21/mujoco-toolbox/commit/187402bb9f945d61a0b62bc732722325752972a3))

- Remove unnecessary --no-root option from Poetry install command
  ([`4fdba6b`](https://github.com/MGross21/mujoco-toolbox/commit/4fdba6bae718ba6b5d6a02233e76164a2f5a4aec))

- Simplify sys.path insertion and reorder import statements in conf.py
  ([`3afa759`](https://github.com/MGross21/mujoco-toolbox/commit/3afa759d24ddc8c3f8ae14b7a2393991be36e722))

- Update __all__ to include Builder and reorder exports
  ([`322f250`](https://github.com/MGross21/mujoco-toolbox/commit/322f2509174c47b4f94963b358e14fb0b34f0a2a))

- Update Builder usage examples for clarity and consistency
  ([`a8d3d4c`](https://github.com/MGross21/mujoco-toolbox/commit/a8d3d4cbe44103f569260ebd3bcf27971bc32ef0))

- Update CI workflow to improve linting and dependency installation steps
  ([`0f7f6e9`](https://github.com/MGross21/mujoco-toolbox/commit/0f7f6e9e92296b6a9526740dd26101d810a56cbb))

- Update commit message to skip CI for auto-fix actions; add README for UR5e model source
  ([`9d050ea`](https://github.com/MGross21/mujoco-toolbox/commit/9d050ea92a6e96f0d0938c1428e4eff33ef18995))

- Update dependency installation in GitHub Actions workflow and remove unnecessary import in Builder
  ([`87b0a77`](https://github.com/MGross21/mujoco-toolbox/commit/87b0a77bb18e2725343997d21fbbc5b7df140d6b))

- Update documentation workflow to ensure proper dependency installation and clean up unnecessary
  steps
  ([`3d6b594`](https://github.com/MGross21/mujoco-toolbox/commit/3d6b5942497f5e8242cb5bae8f30e9abb33bde10))

- Update Poetry version and adjust markers for Python compatibility
  ([`94001cf`](https://github.com/MGross21/mujoco-toolbox/commit/94001cf0b23ef1dbfe12bb804b5b3c31ef130311))

- Update README examples to reflect support for multiple XML files in Builder
  ([`cd39095`](https://github.com/MGross21/mujoco-toolbox/commit/cd39095a247f1bae7294d3b11f811137e0741a56))

- Update sys.path insertion to use absolute path for better compatibility
  ([`68cfa5e`](https://github.com/MGross21/mujoco-toolbox/commit/68cfa5e5617bd4ccff49156905c3ea68bf61ed26))

- Update Wrapper class to accept multiple XML files or Builder instances for model initialization
  ([`173006e`](https://github.com/MGross21/mujoco-toolbox/commit/173006e5d8a9867ec1dad3cd09161fcec256012f))

### Chores

- Auto-fix code quality issues
  ([`a61bd0c`](https://github.com/MGross21/mujoco-toolbox/commit/a61bd0c0c6f8c1b7038e288ff05e61b89cbe0df8))

- Auto-fix code quality issues [skip ci]
  ([`712cd4f`](https://github.com/MGross21/mujoco-toolbox/commit/712cd4f1960ab6ad04401ba7d8ed0f43979e3efd))

### Documentation

- Add warning about zero-release stage and potential API changes in README
  ([`ff7740b`](https://github.com/MGross21/mujoco-toolbox/commit/ff7740bd73627595841a73989cdc94164ab5f9de))


## v5.0.1-rc.1 (2025-04-04)

### Bug Fixes

- Continue linting and type hinting
  ([`432d51f`](https://github.com/MGross21/mujoco-toolbox/commit/432d51ff4c71ceb47cceff57f56b5000f0b9e6c4))

- Frame allocation sizing
  ([`4a87949`](https://github.com/MGross21/mujoco-toolbox/commit/4a879495aee23b8a1a0e8a87766d2276e22d8317))

- Update __exit__ method signature to accept variable arguments
  ([`b70d08b`](https://github.com/MGross21/mujoco-toolbox/commit/b70d08be3ee2cfd6e014b65bed6551606590a67f))

### Chores

- Enable prerelease for dev branch in semantic release configuration
  ([`a2f0833`](https://github.com/MGross21/mujoco-toolbox/commit/a2f0833ce5ba3cdc2b633280b6c28f04076ac4b6))

- Simplify docstring for Builder class in builder.py
  ([`d894296`](https://github.com/MGross21/mujoco-toolbox/commit/d894296f63dda340bb3ba02569f3777fb714fbbf))

- Update .gitignore to clarify cache exclusions
  ([`6488e0f`](https://github.com/MGross21/mujoco-toolbox/commit/6488e0fafd1d005d7a0bae7fec1f4f9a1fa0f8e4))

### Code Style

- Improve docstring formatting and readability in __init__.py
  ([`badb21b`](https://github.com/MGross21/mujoco-toolbox/commit/badb21bc8890c625f9d9bdcaa8ce3d88538fd2bd))

### Documentation

- Rearrange header in README for improved clarity and organization
  ([`9516aa9`](https://github.com/MGross21/mujoco-toolbox/commit/9516aa9f9a2e8692ec0d0a993b78ac8ef00149a3))

- Remove obsolete assets module documentation
  ([`a9a60b9`](https://github.com/MGross21/mujoco-toolbox/commit/a9a60b9ba47d65a7e77c831650d8fe8a9212f22d))

- Update documentation structure and enhance module descriptions
  ([`c94c26f`](https://github.com/MGross21/mujoco-toolbox/commit/c94c26fc409624a46887f67cc27a9dbab385568f))

- Update README example to clarify supported mesh file formats
  ([`06ebe18`](https://github.com/MGross21/mujoco-toolbox/commit/06ebe1833daf69d28923164c9ad4ac89362407e3))

### Refactoring

- Add return type hints for performance functions
  ([`f8c518f`](https://github.com/MGross21/mujoco-toolbox/commit/f8c518f84fbe7f584bbf201a025ffd1a46ce2a8b))

- Move SimulationError and SimulationWarning classes to __init__.py for better organization
  ([`d6e9107`](https://github.com/MGross21/mujoco-toolbox/commit/d6e91078f3ac12de14f47c9553e4d94f29cce17d))

- Rename _initcond to init_conditions for consistency and clarity
  ([`91a8f78`](https://github.com/MGross21/mujoco-toolbox/commit/91a8f78f2a00485e7d7351c16e8de28986d3242e))


## v0.5.0 (2025-04-03)

### Bug Fixes

- Adjust frame count in wrapper to exclude the last frame
  ([`0d5919f`](https://github.com/MGross21/mujoco-toolbox/commit/0d5919f807474fd2f7b7d66a5d05a5296200ad61))

- Update controller method calls and remove unnecessary copy in rendering
  ([`e069007`](https://github.com/MGross21/mujoco-toolbox/commit/e069007ebd81ff3fa799e65976815385a33d494a))

- Update mesh directory path in UR5e model XML
  ([`abd2220`](https://github.com/MGross21/mujoco-toolbox/commit/abd22201e6fac08f22f7a505fe356044675d2bdf))

- Update mujoco_tbx to use keyword arguments for clarity
  ([`54ee0f6`](https://github.com/MGross21/mujoco-toolbox/commit/54ee0f6ae5772b2a2d5eb472037ba4ffdc30d944))

- Update Wrapper to accept Builder instances and adjust README example
  ([`f57d7d3`](https://github.com/MGross21/mujoco-toolbox/commit/f57d7d37dc41ba379d12c5d23db210149665dbc7))

### Chores

- Refactor publish workflow for semantic release and streamline process
  ([`bd45d5d`](https://github.com/MGross21/mujoco-toolbox/commit/bd45d5d1436a5b7d72988fcc553655faafa51988))

- Update pyproject.toml for versioning and dependencies organization
  ([`a4e8f35`](https://github.com/MGross21/mujoco-toolbox/commit/a4e8f356b2ec48270abc2333f2c4ae9a70281481))

- Update version and changelog
  ([`8a88875`](https://github.com/MGross21/mujoco-toolbox/commit/8a888751fb3d6df95924a5aa9e44a73a2b8a2e2a))

- Update version and changelog
  ([`162b9f8`](https://github.com/MGross21/mujoco-toolbox/commit/162b9f8fd3b075a753ca66a338dc7ba2cef17380))

- Update version and changelog
  ([`a9dca24`](https://github.com/MGross21/mujoco-toolbox/commit/a9dca241627468673cb55cf65cae07a1c6b145e3))

- Update version and changelog
  ([`dc98e01`](https://github.com/MGross21/mujoco-toolbox/commit/dc98e01d07baf696121dd158d8bcfcfc3e61bef1))

- Update version and changelog
  ([`aa0f375`](https://github.com/MGross21/mujoco-toolbox/commit/aa0f3755ad18982a6ac63f403aea36118a118aff))

- Update version and changelog
  ([`0814d2a`](https://github.com/MGross21/mujoco-toolbox/commit/0814d2a41fe2f8c40711ed433fca8bdc82efbec9))

- Update version and changelog
  ([`3859b48`](https://github.com/MGross21/mujoco-toolbox/commit/3859b4874bf28673e0d35344f63775262c54bfe3))

- Update version and changelog
  ([`5fe9f0a`](https://github.com/MGross21/mujoco-toolbox/commit/5fe9f0adac5bf572462730bf8000f7162fea42f7))

- Update version and changelog
  ([`76bbd72`](https://github.com/MGross21/mujoco-toolbox/commit/76bbd72bcd894c571827e2d33b11c68e78f9e761))

- Update version to 0.5.0 and enhance changelog
  ([`66e53b1`](https://github.com/MGross21/mujoco-toolbox/commit/66e53b1d3cbae2b6f77af04b94b194b42a707882))

### Documentation

- Enhance README with clearer examples and structure for controllers and merging capabilities
  ([`3b1954c`](https://github.com/MGross21/mujoco-toolbox/commit/3b1954c7d2b553f862cc7e3ee2bea04fa7af5273))

### Features

- Implement Loader class for loading MuJoCo models from XML and URDF files
  ([`b13403b`](https://github.com/MGross21/mujoco-toolbox/commit/b13403b394074cbbf29f61a7a94d2c889600dcf3))

### Refactoring

- Change monitors method to static method in _Platform class
  ([`a0f6e37`](https://github.com/MGross21/mujoco-toolbox/commit/a0f6e37f68dde8eba12ca7b57db6aa27130aa8d3))

- Remove unused scripts
  ([`235aa68`](https://github.com/MGross21/mujoco-toolbox/commit/235aa68ba2a30243b062956691d22ab393d23edb))


## v0.4.5 (2025-03-26)


## v0.4.4 (2025-03-26)

### Bug Fixes

- Streamline version push command and update module docstring formatting
  ([`e6a3276`](https://github.com/MGross21/mujoco-toolbox/commit/e6a3276f8ca7005e9d366bb676e3cb26ee86dfbf))


## v0.4.3 (2025-03-26)

### Bug Fixes

- Enhance release workflow to compare versions and conditionally publish
  ([`3f5e9fa`](https://github.com/MGross21/mujoco-toolbox/commit/3f5e9fa5c9036234cbfb364296a2caaac9e14699))

- Update command to retrieve latest version in release workflow
  ([`cb76cbf`](https://github.com/MGross21/mujoco-toolbox/commit/cb76cbf540e6f25e6c5c29a290769e3921b5e97a))

- Update release workflow to set environment variables and enhance version extraction
  ([`d40a117`](https://github.com/MGross21/mujoco-toolbox/commit/d40a117cd39e5e919b88a8c2be4fa41d1868f79c))

### Chores

- Add conditional check for tag push in release workflow
  ([`1d39ffc`](https://github.com/MGross21/mujoco-toolbox/commit/1d39ffc1271f7445aca25808bd7f9180d190987a))

- Update version and changelog
  ([`aedcbeb`](https://github.com/MGross21/mujoco-toolbox/commit/aedcbeb843d321a65725d53738914de381733b87))

- Update version and changelog
  ([`993108b`](https://github.com/MGross21/mujoco-toolbox/commit/993108bb1ecaa5c1ee90579304e33639ec08c9fe))

- Update version and changelog
  ([`c2109a1`](https://github.com/MGross21/mujoco-toolbox/commit/c2109a1cb336fee9e9e8462efb57fbf37de5e37c))

- Update version and changelog
  ([`13478e2`](https://github.com/MGross21/mujoco-toolbox/commit/13478e239201771654fc9b05845115dc3d5a846e))

- Update version and changelog
  ([`b8baa40`](https://github.com/MGross21/mujoco-toolbox/commit/b8baa40344992ba8884c6f68001346c1a1e84a88))

- Update version and changelog
  ([`2f7e749`](https://github.com/MGross21/mujoco-toolbox/commit/2f7e749a79a2790420ca8a68067cdb0af97117dc))

- Update version and changelog
  ([`08517a8`](https://github.com/MGross21/mujoco-toolbox/commit/08517a84a23bb5983a0e75f896f5e2e4ea299fbf))

- Update version and changelog
  ([`f06826f`](https://github.com/MGross21/mujoco-toolbox/commit/f06826f48bcab145bd0dc1d78f02be432f073033))

- Update version and changelog
  ([`1497a91`](https://github.com/MGross21/mujoco-toolbox/commit/1497a91b9b9ec76fedb79b945f01b9e1bee696be))

- Update version and changelog
  ([`73e8b17`](https://github.com/MGross21/mujoco-toolbox/commit/73e8b178f0881ba818c1759d7a2d78eec5ec05ae))

- **version**: Bump release. remove verbosity
  ([`6609c4c`](https://github.com/MGross21/mujoco-toolbox/commit/6609c4c3c61f19860a93141ab72b9bc0c735903c))


## v0.4.2 (2025-03-26)

### Bug Fixes

- Configure GitHub Token for authentication in publish workflow
  ([`bde07f5`](https://github.com/MGross21/mujoco-toolbox/commit/bde07f50bfd2e8d569e4086d2b007da5b146b4e7))


## v0.4.1 (2025-03-26)

### Bug Fixes

- Update GitHub Actions workflow to trigger on version tags and enhance release process
  ([`4d2d0e3`](https://github.com/MGross21/mujoco-toolbox/commit/4d2d0e3c33faa8908ed63fcc7eb347a006162ec7))

### Chores

- Update publish workflow and changelog handling; remove release trigger
  ([`16668fc`](https://github.com/MGross21/mujoco-toolbox/commit/16668fc8c61d7b27007ca2ab4a0ae5732e03655c))


## v0.4.0 (2025-03-26)

### Bug Fixes

- Update image links in README to use remote paths
  ([`52c2ec6`](https://github.com/MGross21/mujoco-toolbox/commit/52c2ec662dfd051eac39e0781ff9d58fbdae2d0a))

- Update installation instructions in README to clarify upgrade process and add GitHub package
  section
  ([`ca728ef`](https://github.com/MGross21/mujoco-toolbox/commit/ca728ef5226b251981fe0505e1a748c1557b4698))

- Update publish workflow to disable credentials and configure poetry for in-project virtual
  environments
  ([`0ecdb83`](https://github.com/MGross21/mujoco-toolbox/commit/0ecdb83e24d59b654cc2c7fa7f260ae6dbadfa20))

- Update README to correct image syntax for Glovebox
  ([`2b921f7`](https://github.com/MGross21/mujoco-toolbox/commit/2b921f722ea15b70fd53474237208c97ee54f426))

### Features

- Implement script to publish all version tags to PyPI
  ([`67b40ab`](https://github.com/MGross21/mujoco-toolbox/commit/67b40abceefd5298a5bcb054ee4c1d2404da50aa))


## v0.3.4 (2025-03-26)

### Bug Fixes

- Add empty commit to trigger release
  ([`3047c68`](https://github.com/MGross21/mujoco-toolbox/commit/3047c68c50258c6825696cbd7f27e3eec6c39c70))

- Update publish workflow and pyproject.toml for semantic-release and dotenv integration
  ([`337a063`](https://github.com/MGross21/mujoco-toolbox/commit/337a0633d33ea09a32caab060531b213edd830f1))

- **pub**: Update publish workflow for improved semantic-release handling
  ([`78c325e`](https://github.com/MGross21/mujoco-toolbox/commit/78c325ea635108cfea4c458d084b88453ac51803))

### Chores

- Release version 0.3.4
  ([`a65a2f2`](https://github.com/MGross21/mujoco-toolbox/commit/a65a2f28208414b358483325d1e3a07fcf2b91aa))


## v0.3.3 (2025-03-26)


## v0.3.0 (2025-03-25)


## v0.2.3 (2025-03-24)


## v0.2.2 (2025-03-23)


## v0.2.1 (2025-03-22)


## v0.2.0 (2025-03-21)
