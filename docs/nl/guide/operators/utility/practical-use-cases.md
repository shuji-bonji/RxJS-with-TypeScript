---
description: Deze presentatie behandelt praktische gebruiksscenario's van RxJS utility operators (tap, startWith, finalize, delay, timeout, retry, etc.). Praktische patronen die vaak worden gebruikt bij UI-ontwikkeling zoals laadstatusbeheer, reactieve formuliervalidatie, API-aanroepcontrole, foutafhandeling, debugging-ondersteuning, etc. worden geïntroduceerd met TypeScript codevoorbeelden. U leert implementatietechnieken om streamgedrag te controleren en te observeren.
---

# Praktische gebruiksscenario's

## Laadstatus beheren

Dit is een voorbeeld van het gebruik van `tap`, `finalize`, etc. om laadstatus te beheren.

```ts
import { of, throwError } from 'rxjs';
import { tap, delay, finalize, catchError } from 'rxjs';

// UI-elementen
const loadingExample = document.createElement('div');
loadingExample.innerHTML = '<h3>API-aanroep en laadstatusbeheer:</h3>';
document.body.appendChild(loadingExample);

// Laadindicator
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Laden...';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#e3f2fd';
loadingIndicator.style.borderRadius = '5px';
loadingIndicator.style.display = 'none';
loadingExample.appendChild(loadingIndicator);

// Data weergavegebied
const dataContainer = document.createElement('div');
dataContainer.style.marginTop = '10px';
dataContainer.style.padding = '10px';
dataContainer.style.border = '1px solid #ddd';
dataContainer.style.borderRadius = '5px';
dataContainer.style.minHeight = '100px';
loadingExample.appendChild(dataContainer);

// Succesknop
const successButton = document.createElement('button');
successButton.textContent = 'Succesvol verzoek';
successButton.style.marginRight = '10px';
successButton.style.padding = '8px 16px';
loadingExample.insertBefore(successButton, loadingIndicator);

// Faalknop
const failButton = document.createElement('button');
failButton.textContent = 'Mislukt verzoek';
failButton.style.padding = '8px 16px';
loadingExample.insertBefore(failButton, loadingIndicator);

// Simuleer succesvol API-verzoek
function simulateSuccessRequest() {
  return of({
    id: 1,
    name: 'Voorbeelddata',
    description: 'Dit zijn gegevens opgehaald van de API.'
  }).pipe(
    // Toon laden bij start verzoek
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simuleer API-latentie
    delay(1500),
    // Verberg laden altijd bij voltooiing verzoek
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Simuleer mislukt API-verzoek
function simulateFailRequest() {
  return throwError(() => new Error('API-verzoek mislukt')).pipe(
    // Toon laden bij start verzoek
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simuleer API-latentie
    delay(1500),
    // Foutafhandeling
    catchError(error => {
      const errorElement = document.createElement('div');
      errorElement.textContent = `Fout: ${error.message}`;
      errorElement.style.color = 'red';
      dataContainer.appendChild(errorElement);

      return throwError(() => error);
    }),
    // Verberg laden altijd bij voltooiing verzoek
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Succesknop klik
successButton.addEventListener('click', () => {
  // Schakel knoppen uit
  successButton.disabled = true;
  failButton.disabled = true;

  simulateSuccessRequest().subscribe({
    next: data => {
      // Toon data
      const dataElement = document.createElement('div');
      dataElement.innerHTML = `
        <h4>${data.name}</h4>
        <p>${data.description}</p>
        <p><em>ID: ${data.id}</em></p>
      `;
      dataContainer.appendChild(dataElement);
    },
    error: err => {
      console.error('Fout:', err);
    },
    complete: () => {
      // Schakel knoppen weer in
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});

// Faalknop klik
failButton.addEventListener('click', () => {
  // Schakel knoppen uit
  successButton.disabled = true;
  failButton.disabled = true;

  simulateFailRequest().subscribe({
    next: () => {
      // Zal niet slagen, maar voor het geval
    },
    error: () => {
      // Fout al afgehandeld door catchError
      console.log('Foutafhandeling voltooid');
    },
    complete: () => {
      // Schakel knoppen weer in
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});
```

## Formuliervalidatie en verzending

Het volgende is een voorbeeld van het implementeren van formuliervalidatie en verzending met behulp van `startWith`, `tap`, `finalize`, etc.

```ts
import { fromEvent, combineLatest, of } from 'rxjs';
import { map, startWith, debounceTime, tap, finalize, catchError, delay } from 'rxjs';

// Formulier UI
const formExample = document.createElement('div');
formExample.innerHTML = '<h3>Reactief formulier voorbeeld:</h3>';
document.body.appendChild(formExample);

// Maak formulierelementen
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formExample.appendChild(form);

// Naam invoerveld
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Naam: ';
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

// E-mail invoerveld
const emailLabel = document.createElement('label');
emailLabel.textContent = 'E-mailadres: ';
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

// Verzendknop
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Verzenden';
submitButton.style.padding = '8px 16px';
submitButton.disabled = true; // Aanvankelijk uitgeschakeld
form.appendChild(submitButton);

// Resultaat weergavegebied
const formResult = document.createElement('div');
formResult.style.marginTop = '20px';
formResult.style.padding = '10px';
formResult.style.border = '1px solid transparent';
formResult.style.borderRadius = '5px';
formResult.style.display = 'none';
formExample.appendChild(formResult);

// Naam invoervalidatie
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'Naam is verplicht' };
    }
    if (value.length < 2) {
      return { value, valid: false, error: 'Naam moet minimaal 2 tekens bevatten' };
    }
    return { value, valid: true, error: null };
  })
);

// E-mail invoervalidatie
const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'E-mailadres is verplicht' };
    }
    if (!emailRegex.test(value)) {
      return { value, valid: false, error: 'Voer een geldig e-mailadres in' };
    }
    return { value, valid: true, error: null };
  })
);

// Monitor formulierbrede validatiestatus
combineLatest([name$, email$]).pipe(
  map(([nameState, emailState]) => {
    // Is gehele formulier geldig
    const isValid = nameState.valid && emailState.valid;

    // Toon validatiefouten
    nameError.textContent = nameState.error || '';
    emailError.textContent = emailState.error || '';

    return isValid;
  })
).subscribe(isValid => {
  // Schakel verzendknop in/uit
  submitButton.disabled = !isValid;
});

// Formulier verzendverwerking
fromEvent(form, 'submit').pipe(
  tap(event => {
    // Voorkom standaard formulierverzending
    event.preventDefault();

    // Zet op verzendstatus
    submitButton.disabled = true;
    submitButton.textContent = 'Verzenden...';

    // Reset resultaat weergavegebied
    formResult.style.display = 'none';
  }),
  // Haal formulierdata op
  map(() => ({
    name: nameInput.value.trim(),
    email: emailInput.value.trim()
  })),
  // Simuleer API-verzoek
  delay(1500),
  // Keer altijd terug naar verzending voltooid status
  finalize(() => {
    submitButton.textContent = 'Verzenden';
    submitButton.disabled = false;
  }),
  // Foutafhandeling
  catchError(error => {
    formResult.textContent = `Fout: ${error.message}`;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#ffebee';
    formResult.style.borderColor = '#f44336';

    return of(null); // Ga door met stream
  })
).subscribe(data => {
  if (data) {
    // Verzending succesvol
    formResult.innerHTML = `
      <div style="font-weight: bold;">Verzending succesvol!</div>
      <div>Naam: ${data.name}</div>
      <div>E-mail: ${data.email}</div>
    `;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#e8f5e9';
    formResult.style.borderColor = '#4caf50';

    // Reset formulier
    nameInput.value = '';
    emailInput.value = '';
  }
});
```

## Hoe een utility operator te kiezen

| Doel | Operator | Gebruikssituatie |
|------|--------------|---------|
| Bijwerking uitvoeren | `tap` | Debugging, loguitvoer, UI-update, etc. |
| Uitvoervertraging van waarden | `delay` | Animatie, timing aanpassing, etc. |
| Timeout instellingen | `timeout` | Timeout voor API-verzoeken, asynchrone verwerking |
| Verwerking bij voltooiing | `finalize` | Opruimen van resources, laadstatus vrijgeven |
| Beginwaarde instellen | `startWith` | Status initialiseren, placeholders tonen |
| Converteren naar array | `toArray` | Batchverwerking, alle resultaten samen verwerken |
| Opnieuw proberen bij fout | `retry` | Netwerkverzoeken, herstellen van tijdelijke fouten |
| Stream herhalen | `repeat` | Polling, periodieke verwerking |

## Samenvatting

Utility operators zijn belangrijke hulpmiddelen die programmeren in RxJS efficiënter en robuuster maken. De juiste combinatie van deze operators biedt de volgende voordelen:

1. **Gemak van debugging**: Met `tap` kunt u eenvoudig de tussenliggende status van de stream controleren.
2. **Fouttolerantie**: De combinatie van `retry`, `timeout` en `catchError` biedt robuuste foutafhandeling.
3. **Resource beheer**: `finalize` kan worden gebruikt om te zorgen voor correcte resource opruiming.
4. **Verbeterde UI-responsiviteit**: `startWith`, `delay`, etc. kunnen worden gebruikt om de gebruikerservaring te verbeteren.
5. **Verbeterde code leesbaarheid**: Gebruik van utility operators kan bijwerkingen duidelijk scheiden van pure dataconversie.

Deze operators tonen hun ware waarde wanneer ze in combinatie met andere operators worden gebruikt in plaats van alleen. Bij daadwerkelijke applicatieontwikkeling is het gebruikelijk om meerdere operators te combineren om complexe asynchrone verwerkingsstromen te beheren.
