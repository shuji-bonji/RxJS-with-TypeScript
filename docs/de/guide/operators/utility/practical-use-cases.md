---
description: Praktische Anwendungsfälle von RxJS Utility-Operatoren (tap, startWith, finalize, delay, timeout, retry etc.) werden erklärt. Mit TypeScript-Codebeispielen werden praktische Muster vorgestellt, die häufig in der UI-Entwicklung verwendet werden, wie Ladezustandsverwaltung, reaktive Formularvalidierung, API-Aufrufsteuerung, Fehlerbehandlung und Debugging-Unterstützung. Implementierungstechniken zur Steuerung und Beobachtung des Stream-Verhaltens können erlernt werden.
---

# Praktische Anwendungsfälle

## Ladezustandsverwaltung

Beispiel für Verwaltung des Ladezustands mit `tap`, `finalize` etc.

```ts
import { of, throwError } from 'rxjs';
import { tap, delay, finalize, catchError } from 'rxjs';

// UI-Elemente
const loadingExample = document.createElement('div');
loadingExample.innerHTML = '<h3>API-Aufruf und Ladezustandsverwaltung:</h3>';
document.body.appendChild(loadingExample);

// Ladeindikator
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Wird geladen...';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#e3f2fd';
loadingIndicator.style.borderRadius = '5px';
loadingIndicator.style.display = 'none';
loadingExample.appendChild(loadingIndicator);

// Datenanzeigebereich
const dataContainer = document.createElement('div');
dataContainer.style.marginTop = '10px';
dataContainer.style.padding = '10px';
dataContainer.style.border = '1px solid #ddd';
dataContainer.style.borderRadius = '5px';
dataContainer.style.minHeight = '100px';
loadingExample.appendChild(dataContainer);

// Erfolgs-Button
const successButton = document.createElement('button');
successButton.textContent = 'Erfolgreiche Anfrage';
successButton.style.marginRight = '10px';
successButton.style.padding = '8px 16px';
loadingExample.insertBefore(successButton, loadingIndicator);

// Fehler-Button
const failButton = document.createElement('button');
failButton.textContent = 'Fehlgeschlagene Anfrage';
failButton.style.padding = '8px 16px';
loadingExample.insertBefore(failButton, loadingIndicator);

// Simulation erfolgreicher API-Anfrage
function simulateSuccessRequest() {
  return of({
    id: 1,
    name: 'Beispieldaten',
    description: 'Dies sind von der API abgerufene Daten.'
  }).pipe(
    // Ladeanzeige bei Anforderungsstart
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // API-Latenz simulieren
    delay(1500),
    // Ladeanzeige bei Anforderungsabschluss immer ausblenden
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Simulation fehlgeschlagener API-Anfrage
function simulateFailRequest() {
  return throwError(() => new Error('API-Anfrage fehlgeschlagen')).pipe(
    // Ladeanzeige bei Anforderungsstart
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // API-Latenz simulieren
    delay(1500),
    // Fehlerbehandlung
    catchError(error => {
      const errorElement = document.createElement('div');
      errorElement.textContent = `Fehler: ${error.message}`;
      errorElement.style.color = 'red';
      dataContainer.appendChild(errorElement);

      return throwError(() => error);
    }),
    // Ladeanzeige bei Anforderungsabschluss immer ausblenden
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Erfolgs-Button-Klick
successButton.addEventListener('click', () => {
  // Buttons deaktivieren
  successButton.disabled = true;
  failButton.disabled = true;

  simulateSuccessRequest().subscribe({
    next: data => {
      // Datenanzeige
      const dataElement = document.createElement('div');
      dataElement.innerHTML = `
        <h4>${data.name}</h4>
        <p>${data.description}</p>
        <p><em>ID: ${data.id}</em></p>
      `;
      dataContainer.appendChild(dataElement);
    },
    error: err => {
      console.error('Fehler:', err);
    },
    complete: () => {
      // Buttons wieder aktivieren
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});

// Fehler-Button-Klick
failButton.addEventListener('click', () => {
  // Buttons deaktivieren
  successButton.disabled = true;
  failButton.disabled = true;

  simulateFailRequest().subscribe({
    next: () => {
      // Erfolg tritt nicht ein, aber sicherheitshalber
    },
    error: () => {
      // Fehler bereits in catchError behandelt
      console.log('Fehlerbehandlung abgeschlossen');
    },
    complete: () => {
      // Buttons wieder aktivieren
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});
```

## Formularvalidierung und -übermittlung

Beispiel für Implementierung von Formularvalidierung und Übermittlungsverarbeitung mit `startWith`, `tap`, `finalize` etc.

```ts
import { fromEvent, combineLatest, of } from 'rxjs';
import { map, startWith, debounceTime, tap, finalize, catchError, delay } from 'rxjs';

// Formular-UI
const formExample = document.createElement('div');
formExample.innerHTML = '<h3>Beispiel für reaktives Formular:</h3>';
document.body.appendChild(formExample);

// Formularelemente erstellen
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formExample.appendChild(form);

// Namenseingabefeld
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Name: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.padding = '8px';
nameInput.style.width = '100%';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

const nameError = document.createElement('div');
nameError.style.color = 'red';
nameError.style.fontSize = '12px';
nameError.style.marginTop = '-10px';
nameError.style.marginBottom = '15px';
form.appendChild(nameError);

// E-Mail-Eingabefeld
const emailLabel = document.createElement('label');
emailLabel.textContent = 'E-Mail-Adresse: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.padding = '8px';
emailInput.style.width = '100%';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

const emailError = document.createElement('div');
emailError.style.color = 'red';
emailError.style.fontSize = '12px';
emailError.style.marginTop = '-10px';
emailError.style.marginBottom = '15px';
form.appendChild(emailError);

// Senden-Button
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Senden';
submitButton.style.padding = '8px 16px';
submitButton.disabled = true; // Anfangszustand deaktiviert
form.appendChild(submitButton);

// Ergebnisanzeigebereich
const formResult = document.createElement('div');
formResult.style.marginTop = '20px';
formResult.style.padding = '10px';
formResult.style.border = '1px solid transparent';
formResult.style.borderRadius = '5px';
formResult.style.display = 'none';
formExample.appendChild(formResult);

// Namenseingabe-Validierung
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'Name ist erforderlich' };
    }
    if (value.length < 2) {
      return { value, valid: false, error: 'Name muss mindestens 2 Zeichen lang sein' };
    }
    return { value, valid: true, error: null };
  })
);

// E-Mail-Eingabe-Validierung
const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'E-Mail-Adresse ist erforderlich' };
    }
    if (!emailRegex.test(value)) {
      return { value, valid: false, error: 'Bitte geben Sie eine gültige E-Mail-Adresse ein' };
    }
    return { value, valid: true, error: null };
  })
);

// Gesamten Formularvalidierungsstatus überwachen
combineLatest([name$, email$]).pipe(
  map(([nameState, emailState]) => {
    // Ob gesamtes Formular gültig ist
    const isValid = nameState.valid && emailState.valid;

    // Validierungsfehler anzeigen
    nameError.textContent = nameState.error || '';
    emailError.textContent = emailState.error || '';

    return isValid;
  })
).subscribe(isValid => {
  // Senden-Button aktivieren/deaktivieren
  submitButton.disabled = !isValid;
});

// Formularübermittlungsverarbeitung
fromEvent(form, 'submit').pipe(
  tap(event => {
    // Standard-Übermittlung des Formulars verhindern
    event.preventDefault();

    // In Übermittlungszustand versetzen
    submitButton.disabled = true;
    submitButton.textContent = 'Wird gesendet...';

    // Ergebnisanzeigebereich zurücksetzen
    formResult.style.display = 'none';
  }),
  // Formulardaten abrufen
  map(() => ({
    name: nameInput.value.trim(),
    email: emailInput.value.trim()
  })),
  // API-Anfrage simulieren
  delay(1500),
  // Immer in Übermittlungsabschlusszustand zurückkehren
  finalize(() => {
    submitButton.textContent = 'Senden';
    submitButton.disabled = false;
  }),
  // Fehlerbehandlung
  catchError(error => {
    formResult.textContent = `Fehler: ${error.message}`;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#ffebee';
    formResult.style.borderColor = '#f44336';

    return of(null); // Stream fortsetzen
  })
).subscribe(data => {
  if (data) {
    // Übermittlung erfolgreich
    formResult.innerHTML = `
      <div style="font-weight: bold;">Übermittlung erfolgreich!</div>
      <div>Name: ${data.name}</div>
      <div>E-Mail: ${data.email}</div>
    `;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#e8f5e9';
    formResult.style.borderColor = '#4caf50';

    // Formular zurücksetzen
    nameInput.value = '';
    emailInput.value = '';
  }
});
```

## Auswahl von Utility-Operatoren

| Zweck | Operator | Anwendungsfall |
|------|--------------|---------|
| Seiteneffekte ausführen | `tap` | Debugging, Log-Ausgabe, UI-Updates usw. |
| Verzögerte Werteausgabe | `delay` | Animation, Timing-Anpassung usw. |
| Timeout-Einstellung | `timeout` | API-Anfragen, Timeout asynchroner Verarbeitung |
| Verarbeitung bei Abschluss | `finalize` | Ressourcenbereinigung, Ladezustand aufheben |
| Anfangswert festlegen | `startWith` | Zustandsinitialisierung, Platzhalteranzeige |
| Umwandlung in Array | `toArray` | Batch-Verarbeitung, alle Ergebnisse zusammen verarbeiten |
| Wiederholung bei Fehler | `retry` | Netzwerkanfragen, Wiederherstellung von vorübergehenden Fehlern |
| Stream-Wiederholung | `repeat` | Polling, regelmäßige Verarbeitung |

## Zusammenfassung

Utility-Operatoren sind wichtige Werkzeuge, um Programmierung mit RxJS effizienter und robuster zu machen. Durch angemessene Kombination dieser Operatoren können folgende Vorteile erzielt werden:

1. **Leichtes Debugging**: Mit `tap` kann der Zwischenzustand von Streams leicht überprüft werden.
2. **Fehlertoleranz**: Durch Kombination von `retry`, `timeout` und `catchError` wird robuste Fehlerbehandlung möglich.
3. **Ressourcenverwaltung**: Mit `finalize` wird angemessene Ressourcenbereinigung garantiert.
4. **Verbesserte UI-Reaktionsfähigkeit**: Mit `startWith`, `delay` usw. kann die Benutzererfahrung verbessert werden.
5. **Verbesserte Code-Lesbarkeit**: Durch Verwendung von Utility-Operatoren können Seiteneffekte und reine Datentransformation klar getrennt werden.

Diese Operatoren entfalten ihren wahren Wert eher durch Kombination mit anderen Operatoren als durch einzelne Verwendung. In der praktischen Anwendungsentwicklung ist es üblich, mehrere Operatoren zu kombinieren, um komplexe asynchrone Verarbeitungsabläufe zu verwalten.
