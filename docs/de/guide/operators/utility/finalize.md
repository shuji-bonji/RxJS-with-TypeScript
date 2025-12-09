---
description: finalize ist ein RxJS Utility-Operator, der eine Verarbeitung definiert, die immer ausgeführt wird, wenn das Observable abgeschlossen, fehlerhaft oder abgemeldet wird. Er eignet sich optimal für Szenarien, die Aufräumarbeiten am Ende des Streams erfordern, wie Ressourcenfreigabe, Beenden der Ladeanzeige oder Bereinigungsverarbeitung. Er garantiert eine zuverlässige Verarbeitungsausführung ähnlich wie try-finally und hilft, Speicherlecks zu verhindern.
---

# finalize - Verarbeitung bei Abschluss

Der `finalize`-Operator definiert eine Verarbeitung, die **immer aufgerufen wird, wenn das Observable abgeschlossen, fehlerhaft oder abgemeldet wird**.
Er ist optimal für Aufräumarbeiten oder das Deaktivieren von UI-Ladeanzeigen, also für "Verarbeitung, die unbedingt ausgeführt werden muss".

## 🔰 Grundlegende Syntax und Funktionsweise

```ts
import { of } from 'rxjs';
import { finalize } from 'rxjs';

of('Abgeschlossen')
  .pipe(finalize(() => console.log('Stream wurde beendet')))
  .subscribe(console.log);
// Ausgabe:
// Abgeschlossen
// Stream wurde beendet
```

In diesem Beispiel wird nach der Emission eines Werts durch `of()` die Verarbeitung in `finalize` ausgeführt.
Das Merkmal ist, dass sie **sowohl bei `complete` als auch bei `error` sicher aufgerufen wird**.

[🌐 RxJS Offizielle Dokumentation - finalize](https://rxjs.dev/api/index/function/finalize)

## 💡 Typische Anwendungsfälle

Hier ist ein Beispiel für das Umschalten der Ladeanzeige vor und nach dem Stream.

```ts
import { of } from 'rxjs';
import { tap, delay, finalize } from 'rxjs';

let isLoading = false;

of('Daten')
  .pipe(
    tap(() => {
      isLoading = true;
      console.log('Laden gestartet');
    }),
    delay(1000),
    finalize(() => {
      isLoading = false;
      console.log('Laden beendet');
    })
  )
  .subscribe((value) => console.log('Abgerufen:', value));
// Ausgabe:
// Laden gestartet
// Abgerufen: Daten
// Laden beendet
```

## 🧪 Praktisches Codebeispiel (mit UI)

```ts
import { interval } from 'rxjs';
import { take, finalize, tap } from 'rxjs';

// Ausgabebereich
const finalizeOutput = document.createElement('div');
finalizeOutput.innerHTML = '<h3>Beispiel für finalize:</h3>';
document.body.appendChild(finalizeOutput);

// Ladeindikator
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Daten werden geladen...';
loadingIndicator.style.backgroundColor = '#e0f7fa';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.borderRadius = '5px';
finalizeOutput.appendChild(loadingIndicator);

// Fortschrittsanzeige
const progressContainer = document.createElement('div');
progressContainer.style.marginTop = '10px';
finalizeOutput.appendChild(progressContainer);

// Element für Abschlussnachricht
const completionMessage = document.createElement('div');
completionMessage.style.marginTop = '10px';
completionMessage.style.fontWeight = 'bold';
finalizeOutput.appendChild(completionMessage);

// Simulation des Datenabrufs
interval(500)
  .pipe(
    take(5), // 5 Werte abrufen
    tap((val) => {
      const progressItem = document.createElement('div');
      progressItem.textContent = `Element ${val + 1} wird verarbeitet...`;
      progressContainer.appendChild(progressItem);
    }),
    finalize(() => {
      loadingIndicator.style.display = 'none';
      completionMessage.textContent = 'Verarbeitung abgeschlossen!';
      completionMessage.style.color = 'green';
    })
  )
  .subscribe({
    complete: () => {
      const successMsg = document.createElement('div');
      successMsg.textContent = 'Alle Daten wurden erfolgreich geladen.';
      completionMessage.appendChild(successMsg);
    },
  });
```

## ✅ Zusammenfassung

- `finalize` wird **immer ausgeführt, unabhängig von Abschluss, Fehler oder manuellem Beenden**
- Optimal für Bereinigungsverarbeitung oder Beenden der Ladeanzeige
- In Kombination mit anderen Operatoren (`tap`, `delay` usw.) ermöglicht es eine **sichere Aufräumarbeit nach asynchroner Verarbeitung**
