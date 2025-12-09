---
description: concatMap ist ein Transformationsoperator, der jedes Observable nacheinander verarbeitet und auf den Abschluss der vorherigen Verarbeitung wartet. Ideal für Szenarien, in denen die Ausführungsreihenfolge wichtig ist, wie serielle API-Aufrufe oder Reihenfolgegarantie bei Datei-Uploads. Realisiert typsichere asynchrone Verarbeitungsketten durch TypeScript-Typinferenz und erklärt die Unterschiede zu mergeMap und switchMap.
---

# concatMap - Jedes Observable nacheinander ausführen

Der `concatMap`-Operator konvertiert jeden Wert des Eingabestreams in ein Observable und **führt diese nacheinander aus und verbindet sie**.
Das Merkmal ist, dass **das nächste Observable nicht gestartet wird, bis das vorherige Observable abgeschlossen ist**.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  concatMap(value =>
    of(`${value} abgeschlossen`).pipe(delay(1000))
  )
).subscribe(console.log);

// Ausgabe (in Reihenfolge):
// A abgeschlossen
// B abgeschlossen
// C abgeschlossen
```
- Jeder Wert wird in ein Observable konvertiert.
- Das nächste Observable wird erst ausgeführt, nachdem das vorherige Observable abgeschlossen ist.

[🌐 RxJS Offizielle Dokumentation - concatMap](https://rxjs.dev/api/index/function/concatMap)

## 💡 Typische Anwendungsmuster
- Ausführung von API-Anfragen, bei denen die Reihenfolge wichtig ist
- Queue-basierte Task-Verarbeitung
- Steuerung von Animationen oder schrittweisen UIs
- Nachrichtenverarbeitung, bei der die Sendereihenfolge wichtig ist


## 🧠 Praktisches Codebeispiel (mit UI)

Beispiel, bei dem bei jedem Klick auf den Button eine Anfrage ausgelöst wird und die Anfragen immer in Reihenfolge verarbeitet werden.

```ts
import { fromEvent, of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Anfrage senden';
document.body.appendChild(button);

// Ausgabebereich
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Klick-Ereignis
fromEvent(button, 'click')
  .pipe(
    concatMap((_, index) => {
      const requestId = index + 1;
      console.log(`Anfrage${requestId} gestartet`);
      return of(`Antwort${requestId}`).pipe(delay(2000));
    })
  )
  .subscribe((response) => {
    const div = document.createElement('div');
    div.textContent = `✅ ${response}`;
    output.appendChild(div);
  });

```

- Jede Anfrage wird immer in Reihenfolge gesendet und abgeschlossen.
- Die nächste Anfrage wird erst ausgelöst, nachdem die vorherige Anfrage abgeschlossen ist.
