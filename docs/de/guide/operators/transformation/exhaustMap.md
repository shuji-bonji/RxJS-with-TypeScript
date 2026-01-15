---
description: Der exhaustMap-Operator ist ein Transformationsoperator, der neue Eingaben ignoriert, bis das aktuell verarbeitete Observable abgeschlossen ist. Effektiv in Situationen, in denen parallele Ausführung eingeschränkt werden soll, wie Verhinderung von Mehrfachklicks auf Formular-Submit-Button oder Verhinderung von doppelten API-Anfragen.
---

# exhaustMap - Eingaben ignorieren

Der `exhaustMap`-Operator **ignoriert neue Eingaben**, bis das aktuell verarbeitete Observable abgeschlossen ist.
Ideal zur Verhinderung von Doppelklicks oder mehrfachem Senden von Anfragen.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent, of } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$
  .pipe(exhaustMap(() => of('Anfrage abgeschlossen').pipe(delay(1000))))
  .subscribe(console.log);

// Ausgabebeispiel:
// (Nur beim ersten Klick wird nach 1 Sekunde "Anfrage abgeschlossen" ausgegeben)

```

- Nachfolgende Eingaben werden ignoriert, bis die laufende Anfrage abgeschlossen ist.

[🌐 RxJS Offizielle Dokumentation - `exhaustMap`](https://rxjs.dev/api/operators/exhaustMap)

## 💡 Typische Anwendungsmuster

- Verhinderung von Mehrfachklicks auf Formular-Submit-Button
- Verhinderung von Doppel-Anfragen (besonders bei Login- oder Zahlungsprozessen)
- Einzelanzeige-Steuerung von Modalen oder Dialogen

## 🧠 Praktisches Codebeispiel (mit UI)

Beim Klick auf den Sende-Button wird ein Sendeprozess gestartet.
**Mehrfaches Klicken während des Sendens wird ignoriert**, und die nächste Sendung wird erst nach Abschluss der ersten Verarbeitung akzeptiert.

```ts
import { fromEvent } from 'rxjs';
import { exhaustMap, delay } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Button erstellen
const submitButton = document.createElement('button');
submitButton.textContent = 'Senden';
document.body.appendChild(submitButton);

// Ausgabebereich erstellen
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Sendeprozess
fromEvent(submitButton, 'click')
  .pipe(
    exhaustMap(() => {
      output.textContent = 'Wird gesendet...';
      return ajax
        .post('https://jsonplaceholder.typicode.com/posts', {
          title: 'foo',
          body: 'bar',
          userId: 1,
        })
        .pipe(delay(2000)); // 2 Sekunden Sendeverzögerung simulieren
    })
  )
  .subscribe({
    next: (response) => {
      output.textContent = 'Erfolgreich gesendet!';
      console.log('Sendung erfolgreich:', response);
    },
    error: (error) => {
      output.textContent = 'Sendefehler';
      console.error('Sendefehler:', error);
    },
  });

```

- Andere Klicks während eines Button-Klicks werden ignoriert.
- Nach 2 Sekunden wird "Erfolgreich gesendet!" oder "Sendefehler" angezeigt.
