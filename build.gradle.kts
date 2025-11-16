import edu.wpi.first.deployutils.deploy.artifact.JavaArtifact
import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.toolchain.NativePlatforms
import groovy.json.JsonSlurper
import java.util.Date

repositories { gradlePluginPortal() }

val systemProps = System.getProperties()!!

systemProps.setProperty("org.gradle.internal.native.headers.unresolved.dependencies.ignore", "true")

// Add the Gradle plugins we will use.
plugins {
  id("java") // For the Java build
  id("edu.wpi.first.wpilib.repositories.WPILibRepositoriesPlugin") version "2025.+"
  id("edu.wpi.first.GradleRIO") version "2025.3.2" // For the FRC program
  id("com.peterabeles.gversion") version "1.10" // For the build constants
  id("com.diffplug.spotless") version "8.0.0" // For formatting
  id("io.freefair.lombok") version "8.4" // For lombok annotations (@Getter, @Setter)
}

repositories {
  mavenLocal()

  val frcYear = "2025"
  val frcHome: File

  if (org.gradle.internal.os.OperatingSystem.current().isWindows) {
    var publicFolder = System.getenv("PUBLIC")
    if (publicFolder == null) {
      publicFolder = "C:\\Users\\Public"
    }
    val homeRoot = File(publicFolder, "wpilib")
    frcHome = File(homeRoot, frcYear)
  } else {
    val userFolder = System.getProperty("user.home")
    val homeRoot = File(userFolder, "wpilib")
    frcHome = File(homeRoot, frcYear)
  }

  val frcHomeMaven = File(frcHome, "maven")
  maven {
    name = "frcHome"
    url = frcHomeMaven.toURI()
  }

  wpi.vendor.vendorRepos.forEach { maven(it.url) }
}

// Set the target and source Java versions
java.sourceCompatibility = JavaVersion.VERSION_17

java.targetCompatibility = JavaVersion.VERSION_17

// Define where the main class is
val ROBOT_MAIN_CLASS = "org.salemrobotics.frc.Main"

// Define the targets to build for (RoboRIO) and artifacts (deployable files)
// This is added by GradleRIO's backing project DeployUtils
deploy {
  targets {
    create("roborio", getTargetTypeClass("RoboRIO")).apply {
      val props = (this as ExtensionAware).extensions.extraProperties

      props.set("team", project.frc.getTeamOrDefault(6324))
      props.set("debug", project.frc.getDebugOrDefault(false))

      // Artifact for the robot program
      artifacts.create("frcJava", getArtifactTypeClass("FRCJavaArtifact")) {
        // Set the maximum size for the java heap
        val maxJavaHeapSize = 75

        // Configure JVM arguments for the robot program artifact
        props.set(
            "jvmArgs",
            arrayOf(
                "-XX:+UnlockExperimentalVMOptions",
                "-XX:GCTimeRatio=6",
                "-XX:+UseSerialGC",
                "-XX:MaxGCPauseMillis=40",
                // The options below should only be enabled on the RIO 2
                "-Xmx${maxJavaHeapSize}M",
                "-Xms${maxJavaHeapSize}M",
                "-XX:+AlwaysPreTouch",
            ),
        )
      }
      // Artifact for static files in the `deploy/` directory.
      artifacts.create("frcStaticFileDeploy", getArtifactTypeClass("FileTreeArtifact")) {
        props.set("files", project.fileTree("src/main/deploy"))
        props.set("deleteOldFiles", true)
        directory = "/home/lvuser/deploy"
      }
    }
  }
}

val deployArtifact = deploy.targets["roborio"]!!.artifacts["frcJava"]!! as JavaArtifact

// Set to true to use debug for JNI
wpi.java.debugJni = false

// Set this to true to enable desktop support
val includeDesktopSupport = true

// Configuration for AdvantageKit
tasks.register<JavaExec>("replayWatch") {
  mainClass = "org.littletonrobotics.junction.ReplayWatch"
  classpath = sourceSets["main"].runtimeClasspath
}

// Dependencies
dependencies {
  compileOnly("org.jetbrains:annotations:26.0.2-1")

  wpi.java.deps.wpilibAnnotations().forEach { annotationProcessor(it.get()) }
  wpi.java.deps.wpilib().forEach { implementation(it.get()) }
  wpi.java.vendor.java().forEach { implementation(it.get()) }

  val roborio = NativePlatforms.roborio
  val desktop = NativePlatforms.desktop

  // RoboRIO and desktop debug implementations
  wpi.java.deps.wpilibJniDebug(roborio).forEach {
    add("roborioDebug", it.get())
    add("nativeDebug", it.get())
  }
  wpi.java.vendor.jniDebug(roborio).forEach {
    add("roborioDebug", it.get())
    add("nativeDebug", it.get())
  }

  // RoboRIO release implementations
  wpi.java.deps.wpilibJniRelease(desktop).forEach {
    add("roborioRelease", it.get())
    add("nativeRelease", it.get())
  }
  wpi.java.vendor.jniRelease(desktop).forEach {
    add("roborioRelease", it.get())
    add("nativeRelease", it.get())
  }

  wpi.sim.enableDebug().forEach { add("simulationDebug", it.get()) }
  wpi.sim.enableRelease().forEach { add("simulationRelease", it.get()) }

  testImplementation("org.junit.jupiter:junit-jupiter:5.10.1")
  testRuntimeOnly("org.junit.platform:junit-platform-launcher")

  val akitJson =
      JsonSlurper().parse(file("${projectDir}/vendordeps/AdvantageKit.json")) as Map<*, *>
  val version = akitJson["version"]

  annotationProcessor("org.littletonrobotics.akit:akit-autolog:${version}")
}

// Configure tests
tasks.test {
  useJUnitPlatform()
  systemProperty("junit.jupiter.extensions.autodetection.enabled", true)
}

// Configure simulation
wpi.sim.addGui().defaultEnabled = false

wpi.sim.addDriverstation()

// Fat JAR configuration
tasks.jar {
  from(configurations.runtimeClasspath.get().map { if (it.isDirectory) it else zipTree(it) })
  from(sourceSets["main"].allSource)

  manifest(GradleRIOPlugin.javaManifest(ROBOT_MAIN_CLASS))
  duplicatesStrategy = DuplicatesStrategy.INCLUDE
}

// Tell actions how to compile the java code into a fat JAR
val jarTask = tasks.jar.get()

deployArtifact.setJarTask(jarTask)

wpi.java.configureExecutableTasks(jarTask)

wpi.java.configureTestTasks(tasks.test.get())

tasks.withType<JavaCompile>().configureEach {
  // String concatenation optimization
  options.compilerArgs.add("-XDstringConcat=inline")
  // Require GVersion to generate a build file when compiling java
  dependsOn("createVersionFile")
}

// Configure GVersion
gversion {
  srcDir = "src/main/java"
  classPackage = "org.salemrobotics.frc"
  className = "BuildConstants"
  dateFormat = "yyyy-MM-dd HH:mm:ss z"
  timeZone = "America/New_York"
  indent = "\t"
}

fun String.runCommand(dir: File): String =
    ProcessBuilder(*this.split(" ").toTypedArray())
        .directory(dir)
        .redirectOutput(ProcessBuilder.Redirect.PIPE)
        .start()
        .inputStream
        .bufferedReader()
        .readText()

// Auto-commit for event branches
tasks.register("eventDeploy") {
  doLast {
    val invokedDeploy = gradle.startParameter.taskNames.any { "deploy" in it.lowercase() }

    if (!invokedDeploy) {
      println("Not running deploy task, skipping commit")
      return@doLast
    }

    val branch: String
    // Try to get the current branch
    try {
      branch = "git branch --show-current".runCommand(projectDir).trim()
    } catch (e: Exception) {
      // Skip if error occurs (e.g. git not found)
      println("Error occurred running git: " + e.message)
      println("Skipping event commit...")
      return@doLast
    }
    val branchPrefix = "event"

    // Skip if the branch does not start with the event prefix
    if (!branch.startsWith(branchPrefix)) {
      println("Not on an event branch, skipping commit")
      return@doLast
    }

    val commitMsg = "Update at '${Date()}'"

    // Stage changes
    providers.exec {
      workingDir = projectDir
      executable = "git"
      args = listOf("add", "-A")
    }

    // Commit changes (ignore exit if nothing to commit)
    providers.exec {
      workingDir = projectDir
      executable = "git"
      args = listOf("commit", "-m", commitMsg)
      isIgnoreExitValue = true
    }

    println("Committed to branch '${branch}'")
    println("Message: '${commitMsg}'")
  }
}

tasks.named("createVersionFile") { dependsOn("eventDeploy") }

val license =
    """
        /*
         * Copyright (c) 2025 The Blue Devils.
         * This program is free software: you can redistribute it and/or modify
         * it under the terms of the GNU General Public License as published by
         * the Free Software Foundation, either version 3 of the License, or
         * any later version.
         *
         * This program is distributed in the hope that it will be useful,
         * but WITHOUT ANY WARRANTY; without even the implied warranty of
         * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
         * GNU General Public License for more details.
         *
         * You should have received a copy of the GNU General Public License
         * along with this program. If not, see <https://www.gnu.org/licenses/>.
         */
    """
        .trimIndent()

tasks.withType<JavaCompile>() { dependsOn("spotlessApply") }

spotless {
  java {
    target("src/**/*.java")
    toggleOffOn()
    googleJavaFormat("1.22.0").reflowLongStrings()
    trimTrailingWhitespace()
    removeUnusedImports()
    endWithNewline()
    licenseHeader(license, "^package")
  }
  groovyGradle {
    target("*.gradle")
    greclipse()
    trimTrailingWhitespace()
    removeSemicolons()
    endWithNewline()
  }
  kotlinGradle {
    target("*.gradle.kts")
    ktfmt()
    trimTrailingWhitespace()
    endWithNewline()
  }
  json {
    target("src/**/*.json")
    gson()
  }
  kotlin {
    target("src/**/*.kt", "src/**/*.kts")
    ktfmt()
    trimTrailingWhitespace()
    endWithNewline()
    licenseHeader(license, "^package")
  }
}
