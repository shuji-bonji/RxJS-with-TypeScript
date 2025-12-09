---
description: Der mergeMap-Operator konvertiert jeden Wert in ein neues Observable und führt diese gleichzeitig parallel aus und führt sie flach zusammen. Praktisch für mehrere API-Anfragen ohne Warteschlange oder zur Verwaltung verschachtelter asynchroner Verarbeitung.
---

# mergeMap - Jeden Wert in Observable konvertieren und parallel zusammenführen

Der `mergeMap`-Operator (Alias `flatMap`) konvertiert jeden Wert in ein neues Observable und **führt diese parallel flach zusammen**.
Sehr praktisch, wenn Anfragen sofort ausgeführt werden sollen, ohne in eine Warteschlange zu gehen, oder für verschachtelte asynchrone Verarbeitung.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  mergeMap(value =>
    of(`${value} abgeschlossen`).pipe(delay(1000))
  )
).subscribe(console.log);

// Ausgabebeispiel (Reihenfolge nicht garantiert):
// A abgeschlossen
// B abgeschlossen
// C abgeschlossen
```

- Für jeden Wert wird ein neues Observable erzeugt.
- Diese Observables werden **parallel ausgeführt**, und die Ergebnisse werden in beliebiger Reihenfolge ausgegeben.

[🌐 RxJS Offizielle Dokumentation - `mergeMap`](https://rxjs.dev/api/operators/mergeMap)

## 💡 Typische Anwendungsmuster

- API-Anfrage bei jedem Buttonklick senden
- Datei-Upload bei jedem File-Drop-Ereignis starten
- Asynchrone Tasks gleichzeitig durch Benutzerinteraktion auslösen

## 🧠 Praktisches Codebeispiel (mit UI)

Beispiel, das bei jedem Klick auf den Button eine asynchrone Anfrage auslöst (Antwort nach 2 Sekunden).

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Anfrage senden';
document.body.appendChild(button);

// Ausgabebereich
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Klick-Ereignis
fromEvent(button, 'click').pipe(
  mergeMap((_, index) => {
    const requestId = index + 1;
    console.log(`Anfrage${requestId} gestartet`);
    return of(`Antwort${requestId}`).pipe(delay(2000));
  })
).subscribe((response) => {
  const div = document.createElement('div');
  div.textContent = `✅ ${response}`;
  output.appendChild(div);
});
```

- Bei jedem Klick wird sofort eine asynchrone Anfrage ausgelöst.
- **Jede Anfrage wartet individuell 2 Sekunden**, daher erscheinen die Ergebnisse nicht in Ankunftsreihenfolge.
- Ideal, um parallele Verarbeitung zu verstehen.
