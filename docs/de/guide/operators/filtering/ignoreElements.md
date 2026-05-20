---
description: "Der ignoreElements-Operator ist ein RxJS-Filteroperator, der alle Werte ignoriert und nur Fertigstellungen und Fehler durchlässt. Dies ist nützlich, wenn man auf den Abschluss des Prozesses wartet."
---

# ignoreElements - nur Fertigstellungen/Fehler werden zugelassen

Der Operator "ignoreElements" **ignoriert alle Werte**, die von der Quelle Observable ausgegeben werden, und nur **Vollständigkeits- und Fehlermeldungen** werden nachgelagert weitergegeben.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Wert:', value), // Nicht aufgerufen
  complete: () => console.log('Erledigt')
});
// Ausgabe: Erledigt
```

**Ablauf der Operation**:.
1. alle 1, 2, 3, 4 und 5 werden ignoriert
2. nur Abschlussmeldungen werden nachgelagert weitergeleitet

[🌐 Offizielle RxJS Dokumentation - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Typisches Nutzungsmuster.

- **Warte auf Prozessabschluss**: wenn man den Wert nicht braucht und nur den Abschluss wissen will.
- **Nur Nebeneffekte ausführen**: Nebeneffekte mit tap ausführen und Werte ignorieren
- **Fehlerbehandlung**: wenn Sie nur Fehler abfangen wollen
- **Synchronisierung von Sequenzen**: Warten auf den Abschluss mehrerer Prozesse

## 🧠 Praktisches Codebeispiel 1: Warten auf den Abschluss des Initialisierungsprozesses

Dies ist ein Beispiel für das Warten auf den Abschluss mehrerer Initialisierungsprozesse.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// UIErstellt
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Initialisierung der Anwendung';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Funktion zum Hinzufügen des Statusprotokolls
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Initialisierungsprozess1: Datenbankverbindung
const initDatabase$ = from(['DBVerbindung herstellen...', 'Prüfen der Tabelle...', 'DBBereit']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Werte werden ignoriert, nur Fertigstellung gemeldet
);

// Initialisierungsprozess2: Konfigurationsdatei wird gelesen
const loadConfig$ = from(['Konfigurationsdatei wird gelesen...', 'Konfigurationsanalyse in Arbeit...', 'Konfigurationsanwendung abgeschlossen']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Initialisierungsprozess3: Benutzer-Authentifizierung
const authenticate$ = from(['Authentifizierungsinformationen werden überprüft...', 'Token-Prüfung im Gange...', 'Authentifizierung abgeschlossen']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Alle Initialisierungsprozesse werden ausgeführt.
addLog('Initialisierung gestartet...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ Alle Initialisierungsvorgänge sind abgeschlossen.！Die Anwendung kann gestartet werden.';
    addLog('Anwendung gestartet', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Fehler bei der Initialisierung: ${err.message}`;
  }
});
```

- Zu jedem Initialisierungsvorgang wird ein detailliertes Protokoll angezeigt, die Werte werden jedoch ignoriert.
- Wenn alle Prozesse abgeschlossen sind, wird eine Abschlussmeldung angezeigt.

## 🎯 Praktisches Codebeispiel 2: Warten auf den Abschluss des Dateiuploads

Dies ist ein Beispiel für die Anzeige des Upload-Fortschritts mehrerer Dateien, wobei jedoch nur der Abschluss gemeldet wird.

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs';

// UIErstellt
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Datei hochladen';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Upload gestartet';
container.appendChild(button);

const progressArea = document.createElement('div');
progressArea.style.marginTop = '10px';
container.appendChild(progressArea);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.display = 'none';
container.appendChild(result);

interface FileUpload {
  name: string;
  size: number;
}

const files: FileUpload[] = [
  { name: 'document.pdf', size: 2500 },
  { name: 'image.jpg', size: 1800 },
  { name: 'video.mp4', size: 5000 }
];

// Datei-Upload-Prozess (mit Fortschrittsanzeige)
function uploadFile(file: FileUpload) {
  const fileDiv = document.createElement('div');
  fileDiv.style.marginTop = '5px';
  fileDiv.style.padding = '5px';
  fileDiv.style.border = '1px solid #ccc';
  progressArea.appendChild(fileDiv);

  const progressSteps = [0, 25, 50, 75, 100];

  return from(progressSteps).pipe(
    delay(200),
    tap(progress => {
      fileDiv.textContent = `📄 ${file.name} (${file.size}KB) - ${progress}%`;
      if (progress === 100) {
        fileDiv.style.backgroundColor = '#e8f5e9';
      }
    }),
    ignoreElements() // Fortschrittswerte ignoriert, nur Fertigstellung gemeldet
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // Alle Dateien werden nacheinander hochgeladen
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // Max.23 Dateien parallel
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ Upload abgeschlossen</strong><br>
        ${files.length}Eine Datei wurde hochgeladen...
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ Fehler: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- Der Fortschritt jeder Datei wird angezeigt, aber die Fortschrittswerte selbst fließen nicht weiter.
- Eine Abschlussmeldung wird angezeigt, wenn alle Uploads abgeschlossen sind.

## 🆚 Vergleich mit ähnlichen Operatoren

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: Alle Werte ignorieren, Abschluss wird durchgereicht
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('ignoreElements: Erledigt')
});
// Ausgabe: ignoreElements: Erledigt

// filter(() => false): Alle Werte filtern, Vervollständigung durchlassen
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('filter: Erledigt')
});
// Ausgabe: filter: Erledigt

// take(0): Wird sofort vervollständigt
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('take(0): Erledigt')
});
// Ausgabe: take(0): Erledigt
```

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Wert:', value), // Nicht aufgerufen
  complete: () => console.log('Erledigt')
});
// Ausgabe: Erledigt
```

**Empfohlen**: Verwenden Sie `ignoreElements()`, wenn Sie absichtlich alle Werte ignorieren wollen. Die Absicht des Codes wird klar sein.

## 🔄 Behandlung von Fehlerbenachrichtigungen.

ignoreElements" ignoriert die Werte, lässt aber **Fehlerbenachrichtigungen durch**.


```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Fehler tritt auf'))
).pipe(
  ignoreElements()
);

// Erfolgsfall
success$.subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('✅ Erledigt'),
  error: err => console.error('❌ Fehler:', err.message)
});
// Ausgabe: ✅ Erledigt

// Fehlerfall
error$.subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('✅ Erledigt'),
  error: err => console.error('❌ Fehler:', err.message)
});
// Ausgabe: ❌ Fehler: Fehler tritt auf
```

## ⚠️ Hinweise.

### 1. Nebenwirkungen werden durchgeführt

`ignoreElements` ignoriert Werte, aber Seiteneffekte (z.B. `tap`) werden ausgeführt.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Nebeneffekte:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Wert:', v),
  complete: () => console.log('Erledigt')
});
// Ausgabe:
// Nebeneffekte: 1
// Nebeneffekte: 2
// Nebeneffekte: 3
// Erledigt
```

### 2. mit Observable verwenden

Bei der Verwendung mit Infinite Observable dauert das Abonnement ewig, da es nie abgeschlossen wird.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Schlechter Fall: Nicht abgeschlossen
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Erledigt') // Nicht aufgerufen
});

// ✅ Gutes Beispiel: take Abgeschlossen in
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Erledigt') // 5Aufgerufen nach einer Sekunde
});
```

### 3. Typen in TypeScript

Der Rückgabewert von `ignoreElements` ist vom Typ `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// ignoreElements Das Ergebnis von Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value ist vom Typ never Typ, daher wird dieser Block nicht ausgeführt
    console.log(value);
  },
  complete: () => console.log('Nur Vervollständigung')
});
```

### 4. wenn der Abschluss nicht garantiert ist

Wenn die Quelle nicht vollständig ist, wird auch `ignoreElements` nicht vollständig sein.

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs';

// ❌ NEVERwird weder abgeschlossen noch ein Fehler ausgegeben
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Erledigt') // Nicht aufgerufen
});
```

## 💡 Praktische Kombinationsmuster

### Muster 1: Initialisierungssequenz

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Wert:', value), // Nicht aufgerufen
  complete: () => console.log('Erledigt')
});
// Ausgabe: Erledigt
```

### Muster 2: Aufräumvorgang


## 📚 Verwandte Operatoren.

- **[filter](./filter)** - filtert Werte auf der Grundlage von Bedingungen.
- **[take](./take)** - es werden nur die ersten N Werte genommen.
- **[skip](./skip)** - überspringt die ersten N Werte
- **[tap](../utility/tap)** - führt eine Nebenaktion aus

## Zusammenfassung.

Der `ignoreElements`-Operator ignoriert alle Werte und lässt nur Vervollständigungen und Fehler durch.

- ✅ Ideal, wenn nur eine Benachrichtigung über den Abschluss erforderlich ist.
- ✅ Nebeneffekte (TAP) werden ausgeführt
- ✅ Fehlermeldungen werden ebenfalls durchgereicht
- ✅ Klarere Absicht als `filter(() => false)`
- ⚠️ Unendliches Observable wird nicht abgeschlossen
- ⚠️ Der Rückgabetyp ist `Observable<nie>`.
- ⚠️ Wert wird komplett ignoriert, aber Seiteneffekte werden ausgeführt
