---
description: Questa presentazione copre i casi d'uso pratici degli operatori utility RxJS (tap, startWith, finalize, delay, timeout, retry, ecc.). Pattern pratici frequentemente usati nello sviluppo UI come gestione stato di caricamento, validazione form reattiva, controllo chiamate API, gestione errori, assistenza al debugging, ecc. saranno introdotti con esempi di codice TypeScript. Imparerai tecniche di implementazione per controllare e osservare il comportamento degli stream.
---

# Casi d'Uso Pratici

## Gestione Stato di Caricamento

Questo è un esempio di utilizzo di `tap`, `finalize`, ecc. per gestire lo stato di caricamento.

```ts
import { of, throwError } from 'rxjs';
import { tap, delay, finalize, catchError } from 'rxjs';

// Elementi UI
const loadingExample = document.createElement('div');
loadingExample.innerHTML = '<h3>Chiamata API e gestione stato caricamento:</h3>';
document.body.appendChild(loadingExample);

// Indicatore di caricamento
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Caricamento...';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#e3f2fd';
loadingIndicator.style.borderRadius = '5px';
loadingIndicator.style.display = 'none';
loadingExample.appendChild(loadingIndicator);

// Area visualizzazione dati
const dataContainer = document.createElement('div');
dataContainer.style.marginTop = '10px';
dataContainer.style.padding = '10px';
dataContainer.style.border = '1px solid #ddd';
dataContainer.style.borderRadius = '5px';
dataContainer.style.minHeight = '100px';
loadingExample.appendChild(dataContainer);

// Bottone successo
const successButton = document.createElement('button');
successButton.textContent = 'Richiesta riuscita';
successButton.style.marginRight = '10px';
successButton.style.padding = '8px 16px';
loadingExample.insertBefore(successButton, loadingIndicator);

// Bottone fallimento
const failButton = document.createElement('button');
failButton.textContent = 'Richiesta fallita';
failButton.style.padding = '8px 16px';
loadingExample.insertBefore(failButton, loadingIndicator);

// Simula richiesta API riuscita
function simulateSuccessRequest() {
  return of({
    id: 1,
    name: 'Dati di esempio',
    description: 'Questi sono i dati recuperati dall\'API.'
  }).pipe(
    // Mostra caricamento all'inizio della richiesta
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simula latenza API
    delay(1500),
    // Nascondi sempre caricamento al completamento della richiesta
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Simula richiesta API fallita
function simulateFailRequest() {
  return throwError(() => new Error('Richiesta API fallita')).pipe(
    // Mostra caricamento all'inizio della richiesta
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simula latenza API
    delay(1500),
    // Gestione errori
    catchError(error => {
      const errorElement = document.createElement('div');
      errorElement.textContent = `Errore: ${error.message}`;
      errorElement.style.color = 'red';
      dataContainer.appendChild(errorElement);

      return throwError(() => error);
    }),
    // Nascondi sempre caricamento al completamento della richiesta
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Click bottone successo
successButton.addEventListener('click', () => {
  // Disabilita bottoni
  successButton.disabled = true;
  failButton.disabled = true;

  simulateSuccessRequest().subscribe({
    next: data => {
      // Visualizza dati
      const dataElement = document.createElement('div');
      dataElement.innerHTML = `
        <h4>${data.name}</h4>
        <p>${data.description}</p>
        <p><em>ID: ${data.id}</em></p>
      `;
      dataContainer.appendChild(dataElement);
    },
    error: err => {
      console.error('Errore:', err);
    },
    complete: () => {
      // Riabilita bottoni
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});

// Click bottone fallimento
failButton.addEventListener('click', () => {
  // Disabilita bottoni
  successButton.disabled = true;
  failButton.disabled = true;

  simulateFailRequest().subscribe({
    next: () => {
      // Non avrà successo, ma per sicurezza
    },
    error: () => {
      // Errore già gestito da catchError
      console.log('Gestione errore completata');
    },
    complete: () => {
      // Riabilita bottoni
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});
```

## Validazione e Invio Form

Di seguito un esempio di implementazione di validazione e invio form usando `startWith`, `tap`, `finalize`, ecc.

```ts
import { fromEvent, combineLatest, of } from 'rxjs';
import { map, startWith, debounceTime, tap, finalize, catchError, delay } from 'rxjs';

// UI Form
const formExample = document.createElement('div');
formExample.innerHTML = '<h3>Esempio form reattivo:</h3>';
document.body.appendChild(formExample);

// Crea elementi form
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formExample.appendChild(form);

// Campo input nome
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nome: ';
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

// Campo input email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Indirizzo email: ';
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

// Bottone invio
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Invia';
submitButton.style.padding = '8px 16px';
submitButton.disabled = true; // Inizialmente disabilitato
form.appendChild(submitButton);

// Area visualizzazione risultato
const formResult = document.createElement('div');
formResult.style.marginTop = '20px';
formResult.style.padding = '10px';
formResult.style.border = '1px solid transparent';
formResult.style.borderRadius = '5px';
formResult.style.display = 'none';
formExample.appendChild(formResult);

// Validazione input nome
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'Il nome è obbligatorio' };
    }
    if (value.length < 2) {
      return { value, valid: false, error: 'Il nome deve essere di almeno 2 caratteri' };
    }
    return { value, valid: true, error: null };
  })
);

// Validazione input email
const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'L\'indirizzo email è obbligatorio' };
    }
    if (!emailRegex.test(value)) {
      return { value, valid: false, error: 'Per favore inserisci un indirizzo email valido' };
    }
    return { value, valid: true, error: null };
  })
);

// Monitora stato validazione dell'intero form
combineLatest([name$, email$]).pipe(
  map(([nameState, emailState]) => {
    // L'intero form è valido
    const isValid = nameState.valid && emailState.valid;

    // Visualizza errori di validazione
    nameError.textContent = nameState.error || '';
    emailError.textContent = emailState.error || '';

    return isValid;
  })
).subscribe(isValid => {
  // Abilita/disabilita bottone invio
  submitButton.disabled = !isValid;
});

// Elaborazione invio form
fromEvent(form, 'submit').pipe(
  tap(event => {
    // Previeni invio form predefinito
    event.preventDefault();

    // Imposta stato di invio
    submitButton.disabled = true;
    submitButton.textContent = 'Invio in corso...';

    // Resetta area visualizzazione risultato
    formResult.style.display = 'none';
  }),
  // Ottieni dati form
  map(() => ({
    name: nameInput.value.trim(),
    email: emailInput.value.trim()
  })),
  // Simula richiesta API
  delay(1500),
  // Ritorna sempre allo stato invio completato
  finalize(() => {
    submitButton.textContent = 'Invia';
    submitButton.disabled = false;
  }),
  // Gestione errori
  catchError(error => {
    formResult.textContent = `Errore: ${error.message}`;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#ffebee';
    formResult.style.borderColor = '#f44336';

    return of(null); // Continua stream
  })
).subscribe(data => {
  if (data) {
    // Invio riuscito
    formResult.innerHTML = `
      <div style="font-weight: bold;">Invio riuscito!</div>
      <div>Nome: ${data.name}</div>
      <div>Email: ${data.email}</div>
    `;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#e8f5e9';
    formResult.style.borderColor = '#4caf50';

    // Resetta form
    nameInput.value = '';
    emailInput.value = '';
  }
});
```

## Come Scegliere un Operatore Utility

| Scopo | Operatore | Situazione di Utilizzo |
|------|--------------|---------|
| Esecuzione effetti collaterali | `tap` | Debugging, output log, aggiornamento UI, ecc. |
| Ritardo output valori | `delay` | Animazione, regolazione timing, ecc. |
| Impostazioni timeout | `timeout` | Timeout per richieste API, elaborazione asincrona |
| Elaborazione al completamento | `finalize` | Pulizia risorse, rilascio stato caricamento |
| Imposta valore iniziale | `startWith` | Inizializza stato, visualizza placeholder |
| Converti in array | `toArray` | Elaborazione batch, tutti i risultati elaborati insieme |
| Ritenta in caso di errore | `retry` | Richieste di rete, recupero da errori temporanei |
| Ripeti uno stream | `repeat` | Polling, elaborazione periodica |

## Riepilogo

Gli operatori utility sono strumenti importanti che rendono la programmazione in RxJS più efficiente e robusta. La corretta combinazione di questi operatori fornisce i seguenti benefici:

1. **Facilità di Debugging**: Usando `tap`, puoi facilmente controllare lo stato intermedio dello stream.
2. **Tolleranza agli Errori**: La combinazione di `retry`, `timeout` e `catchError` fornisce una gestione errori robusta.
3. **Gestione Risorse**: `finalize` può essere usato per garantire la corretta pulizia delle risorse.
4. **Migliore reattività UI**: `startWith`, `delay`, ecc. possono essere usati per migliorare l'esperienza utente.
5. **Migliore leggibilità del codice**: L'uso degli operatori utility può separare chiaramente gli effetti collaterali dalla pura conversione dati.

Questi operatori dimostrano il loro vero valore quando usati in combinazione con altri operatori piuttosto che da soli. Nello sviluppo di applicazioni reali, è comune combinare più operatori per gestire flussi di elaborazione asincrona complessi.
