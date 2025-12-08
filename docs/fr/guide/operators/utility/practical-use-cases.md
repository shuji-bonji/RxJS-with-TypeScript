---
description: "Explique les cas pratiques d'utilisation des opérateurs utilitaires de RxJS (tap, startWith, finalize, delay, timeout, retry, etc.). Introduit des modèles pratiques fréquemment utilisés dans le développement de l'interface utilisateur, tels que la gestion de l'état de chargement, la validation réactive des formulaires, le contrôle des appels d'API, la gestion des erreurs et le support de débogage avec des exemples de code TypeScript."
---

# Cas d'utilisation pratiques

## Gestion de l'état de chargement

Exemple de gestion de l'état de chargement en utilisant `tap`, `finalize`, etc.

```ts
import { of, throwError } from 'rxjs';
import { tap, delay, finalize, catchError } from 'rxjs';

// Éléments de l'interface utilisateur
const loadingExample = document.createElement('div');
loadingExample.innerHTML = '<h3>Appel d\'API et gestion de l\'état de chargement :</h3>';
document.body.appendChild(loadingExample);

// Indicateur de chargement
const loadingIndicator = document.createElement('div');
loadingIndicator.textContent = 'Chargement...';
loadingIndicator.style.padding = '10px';
loadingIndicator.style.backgroundColor = '#e3f2fd';
loadingIndicator.style.borderRadius = '5px';
loadingIndicator.style.display = 'none';
loadingExample.appendChild(loadingIndicator);

// Zone d'affichage des données
const dataContainer = document.createElement('div');
dataContainer.style.marginTop = '10px';
dataContainer.style.padding = '10px';
dataContainer.style.border = '1px solid #ddd';
dataContainer.style.borderRadius = '5px';
dataContainer.style.minHeight = '100px';
loadingExample.appendChild(dataContainer);

// Bouton de réussite
const successButton = document.createElement('button');
successButton.textContent = 'Requête réussie';
successButton.style.marginRight = '10px';
successButton.style.padding = '8px 16px';
loadingExample.insertBefore(successButton, loadingIndicator);

// Bouton d'échec
const failButton = document.createElement('button');
failButton.textContent = 'Requête échouée';
failButton.style.padding = '8px 16px';
loadingExample.insertBefore(failButton, loadingIndicator);

// Simulation d'une requête API réussie
function simulateSuccessRequest() {
  return of({
    id: 1,
    name: 'Données exemple',
    description: 'Ce sont des données récupérées depuis l\'API.'
  }).pipe(
    // Afficher le chargement au début de la requête
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simuler la latence de l'API
    delay(1500),
    // Toujours cacher le chargement lorsque la requête est terminée
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Simulation d'une requête API échouée
function simulateFailRequest() {
  return throwError(() => new Error('La requête API a échoué')).pipe(
    // Afficher le chargement au début de la requête
    tap(() => {
      loadingIndicator.style.display = 'block';
      dataContainer.innerHTML = '';
    }),
    // Simuler la latence de l'API
    delay(1500),
    // Gestion des erreurs
    catchError(error => {
      const errorElement = document.createElement('div');
      errorElement.textContent = `Erreur : ${error.message}`;
      errorElement.style.color = 'red';
      dataContainer.appendChild(errorElement);

      return throwError(() => error);
    }),
    // Toujours cacher le chargement lorsque la requête est terminée
    finalize(() => {
      loadingIndicator.style.display = 'none';
    })
  );
}

// Clic sur le bouton de réussite
successButton.addEventListener('click', () => {
  // Désactivation des boutons
  successButton.disabled = true;
  failButton.disabled = true;

  simulateSuccessRequest().subscribe({
    next: data => {
      // Affichage des données
      const dataElement = document.createElement('div');
      dataElement.innerHTML = `
        <h4>${data.name}</h4>
        <p>${data.description}</p>
        <p><em>ID : ${data.id}</em></p>
      `;
      dataContainer.appendChild(dataElement);
    },
    error: err => {
      console.error('Erreur :', err);
    },
    complete: () => {
      // Réactiver les boutons
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});

// Clic sur le bouton d'échec
failButton.addEventListener('click', () => {
  // Désactivation des boutons
  successButton.disabled = true;
  failButton.disabled = true;

  simulateFailRequest().subscribe({
    next: () => {
      // N'aboutira jamais, mais juste au cas où
    },
    error: () => {
      // L'erreur a déjà été traitée dans catchError
      console.log('Gestion des erreurs terminée');
    },
    complete: () => {
      // Réactiver les boutons
      successButton.disabled = false;
      failButton.disabled = false;
    }
  });
});
```

## Validation et soumission du formulaire

Exemple d'implémentation de la validation et de la soumission d'un formulaire en utilisant `startWith`, `tap`, `finalize`, etc.

```ts
import { fromEvent, combineLatest, of } from 'rxjs';
import { map, startWith, debounceTime, tap, finalize, catchError, delay } from 'rxjs';

// Interface utilisateur du formulaire
const formExample = document.createElement('div');
formExample.innerHTML = '<h3>Exemple de formulaire réactif :</h3>';
document.body.appendChild(formExample);

// Création d'un élément de formulaire
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formExample.appendChild(form);

// Champ de saisie du nom
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nom : ';
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

// Champ de saisie de l'adresse email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email : ';
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

// Bouton de soumission
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Soumettre';
submitButton.style.padding = '8px 16px';
submitButton.disabled = true; // Initialement désactivé
form.appendChild(submitButton);

// Zone d'affichage du résultat
const formResult = document.createElement('div');
formResult.style.marginTop = '20px';
formResult.style.padding = '10px';
formResult.style.border = '1px solid transparent';
formResult.style.borderRadius = '5px';
formResult.style.display = 'none';
formExample.appendChild(formResult);

// Validation de la saisie du nom
const name$ = fromEvent(nameInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'Le nom est requis' };
    }
    if (value.length < 2) {
      return { value, valid: false, error: 'Le nom doit comporter au moins 2 caractères' };
    }
    return { value, valid: true, error: null };
  })
);

// Validation de la saisie de l'email
const emailRegex = /^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$/;
const email$ = fromEvent(emailInput, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value.trim()),
  startWith(''),
  debounceTime(300),
  map(value => {
    if (!value) {
      return { value, valid: false, error: 'L\'email est requis' };
    }
    if (!emailRegex.test(value)) {
      return { value, valid: false, error: 'Veuillez saisir une adresse email valide' };
    }
    return { value, valid: true, error: null };
  })
);

// Surveiller l'état général de la validation du formulaire
combineLatest([name$, email$]).pipe(
  map(([nameState, emailState]) => {
    // Si l'ensemble du formulaire est valide
    const isValid = nameState.valid && emailState.valid;

    // Affichage des erreurs de validation
    nameError.textContent = nameState.error || '';
    emailError.textContent = emailState.error || '';

    return isValid;
  })
).subscribe(isValid => {
  // Activation/désactivation du bouton de soumission
  submitButton.disabled = !isValid;
});

// Traitement de la soumission du formulaire
fromEvent(form, 'submit').pipe(
  tap(event => {
    // Empêche la soumission du formulaire par défaut
    event.preventDefault();

    // Passer à l'état de soumission
    submitButton.disabled = true;
    submitButton.textContent = 'Soumission...';

    // Réinitialiser la zone d'affichage du résultat
    formResult.style.display = 'none';
  }),
  // Obtenir les données du formulaire
  map(() => ({
    name: nameInput.value.trim(),
    email: emailInput.value.trim()
  })),
  // Simulation d'une requête API
  delay(1500),
  // Retourne toujours à l'état de soumission complète
  finalize(() => {
    submitButton.textContent = 'Soumettre';
    submitButton.disabled = false;
  }),
  // Gestion des erreurs
  catchError(error => {
    formResult.textContent = `Erreur : ${error.message}`;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#ffebee';
    formResult.style.borderColor = '#f44336';

    return of(null); // Continuer le flux
  })
).subscribe(data => {
  if (data) {
    // Succès de la soumission
    formResult.innerHTML = `
      <div style="font-weight: bold;">Soumission réussie !</div>
      <div>Nom : ${data.name}</div>
      <div>Email : ${data.email}</div>
    `;
    formResult.style.display = 'block';
    formResult.style.backgroundColor = '#e8f5e9';
    formResult.style.borderColor = '#4caf50';

    // Réinitialisation du formulaire
    nameInput.value = '';
    emailInput.value = '';
  }
});
```

## Choix des opérateurs utilitaires

| But | Opérateur | Cas d'utilisation |
|-----|-----------|-------------------|
| Exécuter des effets secondaires | `tap` | Débogage, journalisation, mise à jour de l'interface utilisateur, etc. |
| Retarder la sortie des valeurs | `delay` | Animations, ajustements de timing, etc. |
| Définir un délai d'attente | `timeout` | Requêtes API, délais de traitement asynchrone |
| Traitement à la fin | `finalize` | Nettoyage des ressources, suppression de l'état de chargement |
| Définir la valeur initiale | `startWith` | Initialisation de l'état, affichage d'un placeholder |
| Convertir en tableau | `toArray` | Traitement par lots, traitement de tous les résultats ensemble |
| Réessayer en cas d'erreur | `retry` | Requêtes réseau, récupération d'erreurs temporaires |
| Répétition de flux | `repeat` | Polling, traitement périodique |

## Résumé

Les opérateurs utilitaires sont des outils essentiels pour rendre la programmation RxJS plus efficace et plus robuste. En combinant ces opérateurs de manière appropriée, vous pouvez obtenir les avantages suivants :

1. **Débogage facile** : L'utilisation de `tap` vous permet de vérifier facilement les états intermédiaires des flux.
2. **Résilience aux erreurs** : La combinaison de `retry`, `timeout`, `catchError` permet une gestion robuste des erreurs.
3. **Gestion des ressources** : L'utilisation de `finalize` assure un nettoyage correct des ressources.
4. **Amélioration de la réactivité de l'interface utilisateur** : L'utilisation de `startWith`, `delay`, etc. peut améliorer l'expérience de l'utilisateur.
5. **Amélioration de la lisibilité du code** : L'utilisation d'opérateurs utilitaires permet de séparer clairement les effets secondaires des transformations de données pures.

Ces opérateurs sont plus puissants lorsqu'ils sont utilisés en combinaison avec d'autres opérateurs plutôt que seuls. Dans le développement d'applications réelles, il est courant de combiner plusieurs opérateurs pour gérer des flux de traitement asynchrones complexes.
