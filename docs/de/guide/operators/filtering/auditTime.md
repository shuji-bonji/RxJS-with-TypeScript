---
description: "auditTime ist ein RxJS-Filteroperator, der auf eine bestimmte Zeit wartet, wenn ein Wert ausgegeben wird, und den letzten Wert innerhalb dieses Zeitraums ausgibt. Er wird am besten verwendet, wenn Sie regelmäßig den letzten Zustand bei hochfrequenten Ereignissen wie der Verfolgung der Bildlaufposition, der Größenänderung des Fensters, der Mausbewegung usw. abfragen möchten. Es ist wichtig, den Unterschied zwischen diesem Operator und throttleTime und debounceTime zu verstehen und sie entsprechend zu verwenden."
---

# auditTime - letzter Wert, der nach der angegebenen Zeit ausgegeben wurde

Der Operator "auditTime" wartet auf eine **angegebene Zeit**, nachdem ein Wert ausgegeben wurde, und gibt den **letzten Wert** innerhalb dieses Zeitraums aus. Danach wartet er auf den nächsten Wert.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Klick!'));
```

**Ablauf der Operation**:.
1. der erste Klick erfolgt
2. 1 Sekunde warten (Klicks während dieser Zeit werden aufgezeichnet, aber nicht ausgegeben)
3. gibt den letzten Klick nach 1 Sekunde aus
Warten auf den nächsten Klick

[🌐 RxJS offizielle Dokumentation - `auditTime`](https://rxjs.dev/api/operators/auditTime)

## 🆚 Gegensatz zu throttleTime

`throttleTime` und `auditTime` sind ähnlich, unterscheiden sich aber in den Werten, die sie ausgeben.

```ts
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(300).pipe(take(10)); // 0, 1, 2, 3, 4, 5, 6, 7, 8, 9

// throttleTime: Ersten Wert ausgeben
source$.pipe(
  throttleTime(1000)
).subscribe(console.log);
// Ausgeben.: 0, 4, 8(erster Wert der jeweiligen Periode)

// auditTime: Letzten Wert ausgeben
source$.pipe(
  auditTime(1000)
).subscribe(console.log);
// Ausgeben.: 3, 6, 9(letzter Wert jeder Periode)
```

**Zeitlinienvergleich**:.

```
Quelle:     0--1--2--3--4--5--6--7--8--9--|
            |        |        |
throttle:   0--------4--------8------------|
            (Erste)   (Erste)   (Erste)

audit:      -------3--------6--------9----|
                  (Letzter)   (Letzter)   (Letzter)
```

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Klick!'));
```

## 💡 Typisches Nutzungsmuster

1. **Optimierung der Fenstergröße**.


```ts
   import { fromEvent } from 'rxjs';
   import { auditTime } from 'rxjs';

   fromEvent(window, 'resize').pipe(
     auditTime(200) // 200msAbfrage der letzten Größe im Intervall
   ).subscribe(() => {
     console.log(`Größe des Fensters: ${window.innerWidth}x${window.innerHeight}`);
   });
   ```

2. **Verfolgung der Bildlaufposition**
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
     console.log(`Position des Bildlaufs: Y=${position.scrollY}, X=${position.scrollX}`);
   });
   ```

3. **Sanfte Ziehbewegung**
   ```ts
   import { fromEvent } from 'rxjs';
   import { auditTime, map, takeUntil, switchMap } from 'rxjs';

   // Ziehbare Elemente erstellen
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

   // Implementierung von Ziehoperationen
   mouseDown$.pipe(
     switchMap(startEvent => {
       const startX = startEvent.clientX - box.offsetLeft;
       const startY = startEvent.clientY - box.offsetTop;

       return mouseMove$.pipe(
         auditTime(16), // Ungefähr.60FPS(siehe auch16ms) zur Aktualisierung der Position
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

## 🧠 Praktisches Codebeispiel (Mausverfolgung)

Dieses Beispiel verfolgt die Mausbewegungen und zeigt die letzte Position in regelmäßigen Abständen an.

```

ts.
import { fromEvent } from 'rxjs';
importieren { auditTime, map } from 'rxjs';

// Erstellen von UI-Elementen
const container = document.createElement('div');.
container.style.height = '300px';
container.style.border = '2px solid #3498db';
container.style.padding = '20px';
container.style.position = 'relativ';
container.textContent = 'Bitte bewegen Sie die Maus innerhalb dieses Bereichs';
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
dot.style.position = 'absolut';
dot.style.display = 'none';
container.appendChild(dot);

// Mausbewegungs-Ereignis
fromEvent\<MouseEvent>(container, 'mousemove').pipe(
  map(event => {
    const rect = container.getBoundingClientRect();
    return {
      x: event.clientX - rect.left,.
      y: event.clientY - rect.top
    };
  }),
  auditTime(100) // Abrufen der neuesten Position alle 100ms
).subscribe(position => {
  positionDisplay.textContent = `Letzte Position (alle 100ms): X=${position.x.toFixed(0)}, Y=${position.y.toFixed(0)}`;

  // Punkt an die letzte Position verschieben
  dot.style.left = `${Position.x - 5}px`;
  dot.style.top = `${Position.y - 5}px`;
  dot.style.display = 'block';
});

```

Dieser Code ruft nur bei jeder Mausbewegung die letzte Position ab und zeigt sie an, auch wenn die Maus häufig bewegt wird,100msDer Code ruft nur die letzte Position bei jeder Mausbewegung ab und zeigt sie an.

## 🎯 debounceTime Unterschiede zwischen

`auditTime` und `debounceTime` ist, dass**beide den letzten Wert ausgeben**aber die**Das Timing ist völlig unterschiedlich**der letzte Wert ausgegeben wird.

### Der entscheidende Unterschied

| Bediener | Betrieb | die unterschiedliche Nutzung des Systems |
|---|---|---|
| `auditTime(ms)` | Wenn ein Wert eintrifft**msAusgabe immer nach**(auch wenn die Eingabe fortgesetzt wird) | Abtastung in regelmäßigen Abständen |
| `debounceTime(ms)` | **Nachdem die Eingabe gestoppt wurde**msAusgabe danach | Warten auf Abschluss der Eingabe |

### Spezifische Beispiele：Unterschiede in der Sucheingabe

```

ts.
import { fromEvent } from 'rxjs';
import { auditTime, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Suchworteingabe';
document.body.appendChild(input);

// auditTime: Suche auch während der Eingabe alle 300ms ausführen
fromEvent(input, 'input').pipe(
  auditTime(300)
).subscribe(() => {
  console.log('auditTime → Suche:', input.value);
});

// debounceTime: 300ms nach Ende der Eingabe warten, dann Suche ausführen
fromEvent(input, 'input').pipe(
  debounceTime(300)
).subscribe(() => {
  console.log('debounceTime → Suche:', input.value);
});

```

### Unterschiede in der Zeitleiste

Unterschied, wenn ein Benutzer auf "" klicktab'→'abc'→'abcd' beim schnellen Tippen:

```

Eingabe-Ereignis: a--b--c--d------------|
              ↓
auditTime: ------c-----d----------|
            (nach 300 ms) (nach 300 ms)
            → Suche nach 'abc', Suche nach 'abcd' (insgesamt 2 Mal)

debounceTime: --------------------d-|
                              (300 ms nach Stopp)
            → Suche nach "abcd" (insgesamt nur einmal)

```

**Leicht zu merken**:
- **`auditTime`**: 'Regelmäßig geprüft (audit)"→ 'In regelmäßigen Abständen prüfen'
- **`debounceTime`**: 'Warten Sie, bis es ruhig geworden ist (...)'.debounceWarten Sie, bis es ruhig ist.→ 'Warten Sie, bis es ruhig ist'

### Praktische Anwendung

```

ts.
// ✅ auditTime falls erforderlich
// - Verfolgung der Scroll-Position (wir wollen sie regelmäßig erhalten, auch wenn wir die ganze Zeit scrollen)
fromEvent(window, 'scroll').pipe(
  auditTime(100) // Abrufen der neuesten Position alle 100ms
).subscribe(/* ... */);

// ✅ wenn debounceTime angemessen ist.
// - Suchfeld (wir wollen nach Abschluss der Eingabe suchen)
fromEvent(searchInput, 'input').pipe(
  debounceTime(300) // 300ms nach Ende der Eingabe warten
).subscribe(/* ... */);

```

## 📋 Typsichere Verwendung

TypeScript Dies ist ein Beispiel für eine typsichere Implementierung, die die Generika in

```

ts.
import { Observable, fromEvent } from 'rxjs';
importieren { auditTime, map } from 'rxjs';

interface MousePosition {
  x: Zahl;
  y: Zahl;
  timestamp: Zahl; }
}

function trackMousePosition(
  element: HTMLElement,.
  intervalMs: Zahl
): Observable {
  return fromEvent\<MouseEvent>(element, 'mousemove').pipe(
    auditTime(intervalMs),.
    map(event => ({
      x: event.clientX, event.
      y: event.clientY,.
      timestamp: Date.now())
    } as MousePosition))
  );
}

// Beispiel für die Verwendung
const canvas = document.createElement('div');
canvas.style.width = '400px';
canvas.style.height = '300px';
canvas.style.border = '1px solid black';
document.body.appendChild(canvas);

trackMousePosition(canvas, 200).subscribe(position => {
  console.log(`Position: (${position.x}, ${position.y}) bei ${position.timestamp}`);
});

```

## 🔄 auditTime und throttleTime Kombination von

In bestimmten Szenarien können beide kombiniert werden.

```

ts.
import { interval } from 'rxjs';
import { throttleTime, auditTime, take } from 'rxjs';

const source$ = interval(100).pipe(take(50));.

// Reihenfolge von throttleTime → auditTime
source$.pipe(
  throttleTime(1000), // den ersten Wert jede Sekunde durchgeben
  auditTime(500) // dann 500ms warten und den letzten Wert ausgeben
).subscribe(console.log);.

```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Klick!'));
```

ts.
import { fromEvent } from 'rxjs';
importieren { auditTime } from 'rxjs';

// Ein Sucheingabefeld erstellen
const input = document.createElement('input');.
input.type = 'Text';
input.placeholder = 'Suche...' ;
document.body.appendChild(input);

// ❌ Schlechtes Beispiel: auditTime für Sucheingabe verwenden
fromEvent(input, 'input').pipe(
  auditTime(300) // Suche wird alle 300ms während der Eingabe durchgeführt
).subscribe(() => {
  console.log('Suche ausgeführt');
});


```ts
import { fromEvent } from 'rxjs';
import { auditTime } from 'rxjs';

fromEvent(document, 'click').pipe(
  auditTime(1000)
).subscribe(() => console.log('Klick!'));
```

ts.
import { fromEvent } from 'rxjs';
import { debounceTime } from 'rxjs';

// Ein Sucheingabefeld erstellen
const input = document.createElement('input');.
input.type = 'Text';
input.placeholder = 'Suche...' ;
document.body.appendChild(input);

// ✅ Gutes Beispiel: debounceTime für Sucheingabe verwenden
fromEvent(input, 'input').pipe(
  debounceTime(300) // 300ms nach Ende der Eingabe warten, bevor gesucht wird
).subscribe(() => {
  console.log('Suche ausgeführt', input.value);
});


## 🎓 Zusammenfassung

### Wann sollte auditTime verwendet werden.
- ✅ Wenn in regelmäßigen Abständen aktuelle Werte benötigt werden
- ✅ Hochfrequente Ereignisse wie Bildlauf, Größenänderung, Mausbewegung
- ✅ Wenn eine periodische Probenahme erforderlich ist
- ✅ Wenn Sie den neuesten Stand wiedergeben wollen.

### Wenn throttleTime verwendet werden soll.
- ✅ Wenn eine sofortige Reaktion erforderlich ist
- ✅ Wenn die Verarbeitung mit dem ersten Wert beginnen soll
- ✅ Verhinderung von Tastenbetätigungen

### Wann sollte debounceTime verwendet werden.
- ✅ Wenn Sie auf den Abschluss der Eingabe warten wollen
- ✅ Suche, Autovervollständigen
- ✅ Warten Sie, bis der Benutzer aufhört zu tippen.

### Hinweise.
- ⚠️ `auditTime` gibt nur den letzten Wert im Zeitraum aus (Zwischenwerte werden verworfen)
- ⚠️ Nicht sehr effektiv, wenn für kurze Intervalle eingestellt
- ⚠️ `throttleTime` oder `debounceTime` können je nach Anwendung besser geeignet sein

## 🚀 Nächste Schritte.

- **[throttleTime](./throttleTime)** - lernen Sie, wie man den ersten Wert weitergibt.
- **[debounceTime](./debounceTime)** - lernen Sie, wie man Werte ausgibt, nachdem die Eingabe gestoppt wurde.
- **[filter](./filter)** - lernen Sie, wie Sie auf der Grundlage von Bedingungen filtern können.
- **[filtering-operator-practical-use-cases](./practical-use-cases)** - lernen Sie, wie man reale Anwendungsfälle nutzt
