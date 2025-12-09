---
description: "Beschreibt, wie man die neuesten Werte mehrerer Observables mit der combineLatest-Erstellungsfunktion kombiniert. Gibt jedes Mal wenn eine Quelle einen Wert ausgibt die neueste Kombination aus, ideal für die Synchronisierung von UI- und Formulareingaben und die Überwachung mehrerer Bedingungen."
---

# combineLatest - Die neuesten Werte kombinieren

`combineLatest` ist eine Erstellungsfunktion, die **alle neuesten Werte mehrerer Observables zusammenfasst und ausgibt**.
Wann immer ein neuer Wert von einer der Quell-Observables ausgegeben wird, wird das Ergebnis der Kombination aller letzten Werte ausgegeben.

## Grundlegende Syntax und Verwendung

```ts
import { combineLatest, of } from 'rxjs';

const obs1$ = of('A', 'B', 'C');
const obs2$ = of(1, 2, 3);

combineLatest([obs1$, obs2$]).subscribe(([val1, val2]) => {
  console.log(val1, val2);
});

// Ausgabe:
// C 1
// C 2
// C 3
```

- Nachdem jede Observable **mindestens einen Wert** ausgegeben hat, wird der kombinierte Wert ausgegeben.
- Immer wenn ein neuer Wert auf einer der beiden Seiten eingeht, wird das letzte Paar erneut ausgegeben.

[🌐 Offizielle RxJS-Dokumentation - combineLatest](https://rxjs.dev/api/index/function/combineLatest)

## Typische Nutzungsmuster

- **Echtzeit-Validierung von Formulareingaben** (z.B. gleichzeitige Überwachung von Name und E-Mail-Adresse)
- **Zustandssynchronisation mehrerer Streams** (z.B. Integration von Sensorwerten und Gerätestatus)
- **Datenabruf mit Abhängigkeiten** (z.B. Kombination von Benutzer-ID und Konfigurations-ID)

## Praktisches Code-Beispiel (mit UI)

Kombiniert immer den aktuellen Status der beiden Eingabefelder eines Formulars.

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatest Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Formularfelder erstellen
const nameInput = document.createElement('input');
nameInput.placeholder = 'Name eingeben';
document.body.appendChild(nameInput);

const emailInput = document.createElement('input');
emailInput.placeholder = 'E-Mail eingeben';
document.body.appendChild(emailInput);

// Observable für jede Eingabe
const name$ = fromEvent(nameInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

const email$ = fromEvent(emailInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  startWith('')
);

// Neueste Eingabewerte kombinieren
combineLatest([name$, email$]).subscribe(([name, email]) => {
  output.innerHTML = `
    <div><strong>Name:</strong> ${name}</div>
    <div><strong>E-Mail:</strong> ${email}</div>
  `;
});
```

- Bei der Eingabe in eines der beiden Felder werden sofort die **letzten beiden Eingabezustände angezeigt**.
- Mit `startWith('')` wird das kombinierte Ergebnis von Anfang an erhalten.

## Verwandte Operatoren

- **[combineLatestWith](/de/guide/operators/combination/combineLatestWith)** - Pipeable Operator Version (wird in der Pipeline verwendet)
- **[withLatestFrom](/de/guide/operators/combination/withLatestFrom)** - Wird nur durch den Hauptstrom ausgelöst
- **[zip](/de/guide/creation-functions/combination/zip)** - Paart entsprechende Werte (Erstellungsfunktion)
