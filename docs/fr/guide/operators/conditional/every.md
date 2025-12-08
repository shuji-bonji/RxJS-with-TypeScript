---
description: "L'opérateur every permet d'évaluer si toutes les valeurs satisfont une condition spécifiée et de renvoyer false dès que la condition n'est pas satisfaite (court-circuit). La validation, les contrôles de qualité des données et le traitement des flux équivalents à Array.every() sont mis en œuvre de manière type-safe en TypeScript."
---

# every - Vérifier que toutes les valeurs remplissent la condition

L'opérateur `every` évalue si chaque valeur émise par l'Observable source satisfait la condition spécifiée et
**termine en retournant `false` dès qu'une condition n'est pas remplie**. Si toutes les conditions sont satisfaites, `true` est retourné.

## 🔰 Syntaxe et comportement de base

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 6, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Sortie : true
```

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 5, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Sortie : false (s'arrête à 5)
```

[🌐 Documentation officielle RxJS - every](https://rxjs.dev/api/index/function/every)

## 💡 Exemples d'utilisation typiques

- **Vérification de validation** : vérifie que toutes les conditions ont été remplies
- **Validation d'entrée par lot** : situations où plusieurs valeurs sont évaluées ensemble
- Contrairement au filtrage de tableaux, efficace pour **vérifier la satisfaction de l'ensemble en une seule fois**

## 🧪 Exemples de code pratiques (avec interface utilisateur)

### ✅ 1. Déterminer si tous les éléments d'un tableau sont pairs

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>Exemple de l\'opérateur every :</h3>';
document.body.appendChild(container);

const allEvenButton = document.createElement('button');
allEvenButton.textContent = 'Nombres pairs uniquement [2, 4, 6, 8]';
container.appendChild(allEvenButton);

const someOddButton = document.createElement('button');
someOddButton.textContent = 'Contient des impairs [2, 4, 5, 8]';
container.appendChild(someOddButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

allEvenButton.addEventListener('click', () => {
  result.textContent = 'Évaluation en cours...';
  from([2, 4, 6, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `Tous pairs ? : ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});

someOddButton.addEventListener('click', () => {
  result.textContent = 'Évaluation en cours...';
  from([2, 4, 5, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `Tous pairs ? : ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});
```

### ✅ 2. Utilisation dans la validation de formulaires

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith, every, tap } from 'rxjs';

// Créer les éléments d'interface utilisateur
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Validation de formulaire avec every :</h3>';
document.body.appendChild(formContainer);

// Créer le formulaire
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formContainer.appendChild(form);

// Entrée du nom
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nom : ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.width = '100%';
nameInput.style.padding = '5px';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

// Entrée de l'âge
const ageLabel = document.createElement('label');
ageLabel.textContent = 'Âge : ';
ageLabel.style.display = 'block';
ageLabel.style.marginBottom = '5px';
form.appendChild(ageLabel);

const ageInput = document.createElement('input');
ageInput.type = 'number';
ageInput.min = '0';
ageInput.style.width = '100%';
ageInput.style.padding = '5px';
ageInput.style.marginBottom = '15px';
form.appendChild(ageInput);

// Entrée de l'email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email : ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.width = '100%';
emailInput.style.padding = '5px';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

// Bouton de soumission
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Envoyer';
submitButton.disabled = true;
form.appendChild(submitButton);

// Message de validation
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Validation du nom
const nameValid$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return value.length >= 2;
  }),
  startWith(false)
);

// Validation de l'âge
const ageValid$ = fromEvent(ageInput, 'input').pipe(
  map((event) => {
    const value = Number((event.target as HTMLInputElement).value);
    return !isNaN(value) && value > 0 && value < 120;
  }),
  startWith(false)
);

// Validation de l'email
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const emailValid$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return emailRegex.test(value);
  }),
  startWith(false)
);

// Validation de tous les champs
combineLatest([nameValid$, ageValid$, emailValid$])
  .pipe(
    // tap((v) => console.log(v)),
    map((validList) => validList.every((v) => v === true))
  )
  .subscribe((allValid) => {
    submitButton.disabled = !allValid;
    if (allValid) {
      validationMessage.textContent = '';
    } else {
      validationMessage.textContent =
        'Veuillez remplir tous les champs correctement';
    }
  });

// Soumission du formulaire
form.addEventListener('submit', (event) => {
  event.preventDefault();

  const formData = {
    name: nameInput.value,
    age: ageInput.value,
    email: emailInput.value,
  };

  validationMessage.textContent = 'Le formulaire a été envoyé !';
  validationMessage.style.color = 'green';

  console.log('Données envoyées :', formData);
});

```
