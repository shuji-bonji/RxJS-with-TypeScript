---
description: auditTime ist ein RxJS-Filteroperator, der nach der Ausgabe eines Werts die angegebene Zeit wartet und den letzten Wert in diesem Zeitraum ausgibt. Ideal für hochfrequente Ereignisse wie Scroll-Positionsverfolgung, Fenstergrößenänderung oder Mausbewegung, wenn Sie den neuesten Zustand regelmäßig samplen möchten. Es ist wichtig, die Unterschiede zu throttleTime und debounceTime zu verstehen und sie angemessen einzusetzen.
---

# auditTime - Letzter Wert pro Periode

Der `auditTime`-Operator **wartet die angegebene Zeit**, nachdem ein Wert ausgegeben wurde, und gibt den **letzten Wert** in diesem Zeitraum aus. Danach wartet er auf den nächsten Wert.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Klick!'));
```

**Ablauf**:
1. Der erste Klick erfolgt
2. 1 Sekunde warten (Klicks in dieser Zeit werden aufgezeichnet, aber nicht ausgegeben)
3. Nach 1 Sekunde den letzten Klick ausgeben
4. Auf den nächsten Klick warten

[🌐 RxJS Offizielle Dokumentation - `auditTime`](https://rxjs.dev/api/operators/auditTime)


## 🆚 Vergleich mit throttleTime

`throttleTime` und `auditTime` sind ähnlich, geben aber unterschiedliche Werte aus.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Gibt den ersten Wert aus
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Ausgabe: 0, 4, 8 (erster Wert jedes Zeitraums)

// auditTime: Gibt den letzten Wert aus
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Ausgabe: 3, 6, 9 (letzter Wert jedes Zeitraums)
```

**Timeline-Vergleich**:
```
Quelle:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (erster) (erster) (erster)

audit:      -------3--------6--------9----|
                  (letzter) (letzter) (letzter)
```

| Operator | Ausgegebener Wert | Ausgabezeitpunkt | Anwendungsfall |
|---|---|---|---|
| `throttleTime(ms)` | **Erster** Wert des Zeitraums | Bei Wertempfang | Sofortige Reaktion erforderlich |
| `auditTime(ms)` | **Letzter** Wert des Zeitraums | Bei Zeitraumende | Neuester Zustand erforderlich |
| `debounceTime(ms)` | **Letzter** Wert nach Ruhe | Nach Eingabestopp | Auf Eingabeabschluss warten |


## 💡 Typische Anwendungsmuster

1. **Fenstergrößenänderung optimieren**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // Neueste Größe alle 200ms abrufen
   ).subscribe(() => {
     console.log(`Fenstergröße: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Scroll-Position verfolgen**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map } from 'rxjs';

   fromEvent(window, 'scroll').pipe(
     auditTime(100),
     map(() => ({
       scrollY: window.scrollY,
       scrollX: window.scrollX
     }))
   ).subscribe(position => {
     console.log(`Scroll-Position: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```

3. **Sanftes Drag & Drop**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Ziehbares Element erstellen
   const box = document.createElement('div');
   box.style.width = '100px';
   box.style.height = '100px';
   box.style.backgroundColor = '#3498db';
   box.style.position = 'absolute';
   box.style.cursor = 'move';
   box.style.left = '100px';
   box.style.top = '100px';
   box.textContent = 'Ziehen';
   box.style.display = 'flex';
   box.style.alignItems = 'center';
   box.style.justifyContent = 'center';
   box.style.color = 'white';
   document.body.appendChild(box);

   const mouseDown$ = fromEvent<MouseEvent>(box, 'mousedown');
   const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove');
   const mouseUp$ = fromEvent<MouseEvent>(document, 'mouseup');

   // Drag-Implementierung
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Position ca. alle 60FPS (16ms) aktualisieren
         map(moveEvent => ({
           x: moveEvent.clientX - startX,
           y: moveEvent.clientY - startY
         })),
         takeUntil(mouseUp$)
       );
     })
   ).subscribe(position => {
     box.style.left = `${position.x}px`;
     box.style.top = `${position.y}px`;
   });
   ```


## 🧠 Praktisches Codebeispiel (Maus-Tracking)

Ein Beispiel, das Mausbewegungen verfolgt und die neueste Position in regelmäßigen Abständen anzeigt.

```ts
import { fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

// UI-Elemente erstellen
const container = document.createElement('div');
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relative';
container.textContent = 'Bewegen Sie die Maus in diesem Bereich';
document.body.appendChild(container);

const positionDisplay = document.createElement('div');
positionDisplay.style.marginTop = '10px';
positionDisplay.style.fontFamily = 'monospace';
document.body.appendChild(positionDisplay);

const dot = document.createElement('div');
dot.style.width = '10px';
dot.style.height = '10px';
dot.style.borderRadius = '50%';
dot.style.backgroundColor = '#e74c3c';
dot.style.position = 'absolute';
dot.style.display = 'none';
container.appendChild(dot);

// Mausbewegungsereignis
fromEvent<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Neueste Position alle 100ms abrufen
).subscribe(position => {
  positionDisplay.textContent = `Neueste Position (100ms-Intervall): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Punkt zur neuesten Position bewegen
  dot.style.left = `${position.x - 5}px`;
  dot.style.top = `${position.y - 5}px`;
  dot.style.display = 'block';
});
```

Dieser Code erfasst auch bei häufiger Mausbewegung nur alle 100ms die neueste Position und zeigt sie an.


## 🎯 Unterschied zu debounceTime

`auditTime` und `debounceTime` **geben beide den letzten Wert aus**, aber **das Timing ist völlig unterschiedlich**.

### Entscheidender Unterschied

| Operator | Verhalten | Einsatz |
|---|---|---|
| `auditTime(ms)` | **Gibt nach ms Wert aus** (auch wenn Eingabe fortgesetzt wird) | Regelmäßiges Sampling gewünscht |
| `debounceTime(ms)` | Gibt ms **nach Eingabestopp** aus | Auf Eingabeabschluss warten |

### Konkretes Beispiel: Unterschied bei Sucheingabe

```ts
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Suchbegriff eingeben';
document.body.appendChild(input);

// auditTime: Suche auch während Eingabe alle 300ms ausführen
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Suche:', input.value);
});

// debounceTime: Suche 300ms nach Eingabestopp ausführen
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Suche:', input.value);
});
```

### Unterschied in Timeline

Wenn der Benutzer schnell „ab" → „abc" → „abcd" eingibt:

```
Eingabeereignis:   a--b--c--d------------|
              ↓
auditTime:    ------c-----d----------|
            (nach 300ms) (nach 300ms)
            → Suche mit „abc", Suche mit „abcd" (insgesamt 2 Mal)

debounceTime: --------------------d-|
                              (300ms nach Stopp)
            → Suche mit „abcd" (nur 1 Mal)
```

**Einfache Eselsbrücke**:
- **`auditTime`**: „Regelmäßig prüfen (audit)" → In festen Intervallen überprüfen
- **`debounceTime`**: „Auf Ruhe warten (debounce)" → Warten bis es ruhig wird

### Praktische Unterscheidung

```ts
// ✅ auditTime ist geeignet
// - Scroll-Positionsverfolgung (auch bei kontinuierlichem Scrollen regelmäßig abrufen)
fromEvent(window, 'scroll').pipe(
  auditTime(100)  // Neueste Position alle 100ms abrufen
).subscribe(/* ... */);

// ✅ debounceTime ist geeignet
// - Suchfeld (nach abgeschlossener Eingabe suchen)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300)  // 300ms nach Eingabestopp warten
).subscribe(/* ... */);
```


## 📋 Typsichere Verwendung

Beispiel für typsichere Implementierung mit TypeScript-Generics.

```ts
import { Observable, fromEvent } from 'rxjs';
import { auditTime, map } from 'rxjs';

interface MousePosition {
  x: number;
  y: number;
  timestamp: number;
}

function trackMousePosition(
  element: HTMLElement,
  intervalMs: number
): Observable<MousePosition> {
  return fromEvent<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs),
    map(event => ({
      x: event.clientX,
      y: event.clientY,
      timestamp: Date.now()
    } as MousePosition))
  );
}

// Verwendungsbeispiel
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px solid black';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`Position: (${position.x}, ${position.y}) um ${position.timestamp}`);
});
```


## 🔄 Kombination von auditTime und throttleTime

In bestimmten Szenarien können beide kombiniert werden.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(100).pipe(take(50));

// Reihenfolge throttleTime → auditTime
source$.pipe(
  throttleTime(1000),  // Ersten Wert alle 1 Sekunde durchlassen
  auditTime(500)       // Danach 500ms warten und letzten Wert ausgeben
).subscribe(console.log);
```


## ⚠️ Häufige Fehler

> [!WARNING]
> `auditTime` und `debounceTime` verhalten sich unterschiedlich. Bei Sucheingaben oder wenn Sie **auf Eingabestopp warten** möchten, verwenden Sie `debounceTime`. `auditTime` gibt auch während der Eingabe in festen Intervallen Werte aus.

### Falsch: auditTime und debounceTime verwechselt

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

// Sucheingabefeld erstellen
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Suchen...';
document.body.appendChild(input);

// ❌ Schlechtes Beispiel: auditTime für Sucheingabe verwenden
fromEvent(input, 'input').pipe(
  auditTime(300) // Suche wird auch während Eingabe alle 300ms ausgeführt
).subscribe(() => {
  console.log('Suche ausführen');
});
```

### Richtig: debounceTime verwenden

```ts
import { fromEvent } from 'rxjs';
import { debounceTime } from 'rxjs';

// Sucheingabefeld erstellen
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Suchen...';
document.body.appendChild(input);

// ✅ Gutes Beispiel: debounceTime für Sucheingabe verwenden
fromEvent(input, 'input').pipe(
  debounceTime(300) // 300ms nach Eingabestopp mit Suche beginnen
).subscribe(() => {
  console.log('Suche ausführen', input.value);
});
```


## 🎓 Zusammenfassung

### Wann auditTime verwenden
- ✅ Wenn neuester Wert in festen Intervallen benötigt wird
- ✅ Hochfrequente Ereignisse wie Scrollen, Größenänderung, Mausbewegung
- ✅ Wenn regelmäßiges Sampling benötigt wird
- ✅ Wenn neuester Zustand widergespiegelt werden soll

### Wann throttleTime verwenden
- ✅ Wenn sofortige Reaktion benötigt wird
- ✅ Wenn Verarbeitung mit erstem Wert beginnen soll
- ✅ Verhinderung von Mehrfachklicks

### Wann debounceTime verwenden
- ✅ Wenn auf Eingabeabschluss gewartet werden soll
- ✅ Suche, Autovervollständigung
- ✅ Warten bis Benutzer Eingabe stoppt

### Hinweise
- ⚠️ `auditTime` gibt nur letzten Wert des Zeitraums aus (Zwischenwerte werden verworfen)
- ⚠️ Bei kurzen Intervallen ist der Effekt gering
- ⚠️ Je nach Zweck können `throttleTime` oder `debounceTime` geeigneter sein


## 🚀 Nächste Schritte

- **[throttleTime](./throttleTime)** - Methode zum Durchlassen des ersten Werts lernen
- **[debounceTime](./debounceTime)** - Methode zum Ausgeben von Werten nach Eingabestopp lernen
- **[filter](./filter)** - Methode zum Filtern basierend auf Bedingungen lernen
- **[Praktische Beispiele für Filteroperatoren](./practical-use-cases)** - Reale Anwendungsfälle lernen
